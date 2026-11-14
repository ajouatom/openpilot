use super::{
    device::{Device, Driver, Linux},
    graph::Call,
    Dispatch, ProgramImage, QcomBundle,
};
use crate::{
    entrypoint,
    graph::{Binding, View},
    Error,
};
use std::path::Path;

#[cfg(test)]
mod tests;

enum Step {
    Gpu(Vec<u32>),
    Copy { source: usize, destination: usize },
}

/// A kernel call's argument block, kept so an input can be rebound.
struct KernelArgs {
    kernel: usize,
    views: Vec<usize>,
    scalars: Vec<u32>,
    buffer: usize,
}

struct Model<D: Driver> {
    bundle: QcomBundle,
    device: Device<D>,
    allocations: Vec<usize>,
    steps: Vec<Step>,
    entries: Vec<Vec<Step>>,
    kernel_args: Vec<KernelArgs>,
    /// Graph allocation -> GPU address of the external memory bound to it.
    external: Vec<Option<u64>>,
}

pub struct QcomModel(Model<Linux>);

impl QcomModel {
    /// # Safety
    /// The bundle must contain trusted QCOMCL a630 kernels matching the declared
    /// argument order, image layout, launch sizes and buffer bounds. Kernel code
    /// can access CPU-mapped memory; checksums do not authenticate that code.
    pub unsafe fn load(directory: &Path, priority: u8) -> Result<Self, Error> {
        let bundle = QcomBundle::load(directory)?;
        Ok(Self(Model::new(bundle, Linux::open(priority)?)?))
    }

    /// Construct from the already loaded, package-bound snapshot without
    /// re-reading mutable graph paths before opening the GPU.
    ///
    /// # Safety
    /// The bundle must contain authenticated QCOMCL a630 kernels whose argument
    /// order, layouts, launch sizes and bounds satisfy `load`'s trust contract.
    pub unsafe fn from_bundle(bundle: QcomBundle, priority: u8) -> Result<Self, Error> {
        Ok(Self(Model::new(bundle, Linux::open(priority)?)?))
    }

    pub fn write_input(&mut self, name: &str, bytes: &[u8]) -> Result<(), Error> {
        self.0.write_input(name, bytes)
    }
    pub fn read_output(&self, name: &str) -> Result<&[u8], Error> {
        self.0.read_output(name)
    }
    /// Let the GPU read input `name` straight from caller memory instead of a
    /// copy, as carrot's modeld does with `Tensor.from_blob` on VisionIPC
    /// buffers. A later `write_input` of `name` restores the internal buffer.
    ///
    /// # Safety
    /// `address..address + len` must stay mapped, at the same address, for
    /// this model's lifetime; the GPU reads it during every later run that
    /// uses `name`.
    pub unsafe fn bind_input(&mut self, name: &str, address: usize, len: usize) -> Result<(), Error> {
        self.0.bind_input(name, address as u64, len)
    }
    pub fn run(&mut self) -> Result<(), Error> {
        self.0.run()
    }
    pub fn run_entry(&mut self, name: &str) -> Result<(), Error> {
        self.0.run_entry(name)
    }
    pub fn reset_mutable(&mut self) -> Result<(), Error> {
        self.0.reset_mutable()
    }
}

impl<D: Driver> Model<D> {
    fn new(bundle: QcomBundle, driver: D) -> Result<Self, Error> {
        let mut device = Device::new(driver);
        let mut allocations = Vec::with_capacity(bundle.graph.allocations.len());
        for allocation in &bundle.graph.allocations {
            let buffer = device.allocate(allocation.bytes)?;
            if let Some(offset) = allocation.weight_offset {
                device.write(
                    buffer,
                    0,
                    &bundle.weights[offset..offset + allocation.bytes],
                )?;
            }
            allocations.push(buffer);
        }
        let stack_bytes = bundle
            .programs
            .iter()
            .map(|program| program.hw_stack_offset as usize * 4)
            .max()
            .unwrap_or(4096);
        let stack_buffer = device.allocate(stack_bytes)?;
        let border_buffer = device.allocate(4096)?;
        let dummy_buffer = device.allocate(4096)?;
        let stack = device.address(stack_buffer, 0, stack_bytes)?;
        let border = device.address(border_buffer, 0, 4096)?;
        let dummy = device.address(dummy_buffer, 0, 4096)?;
        let mut program_addresses = Vec::with_capacity(bundle.programs.len());
        for program in &bundle.programs {
            let buffer = device.allocate(program.image().len())?;
            device.write(buffer, 0, program.image())?;
            program_addresses.push(device.address(buffer, 0, program.image().len())?);
        }
        let mut calls = Vec::with_capacity(bundle.graph.calls.len());
        let mut kernel_args = Vec::new();
        let mut command_words = 0_usize;
        for call in &bundle.graph.calls {
            match call {
                Call::Copy {
                    source,
                    destination,
                } => {
                    calls.push(Step::Copy {
                        source: *source,
                        destination: *destination,
                    });
                }
                Call::Kernel {
                    kernel,
                    views,
                    scalars,
                    global,
                    local,
                } => {
                    let program = &bundle.programs[*kernel];
                    let mut arguments = Vec::new();
                    for (view, specs) in views.iter().zip(&bundle.graph.kernels[*kernel].arguments)
                    {
                        let view = &bundle.graph.views[*view];
                        let address = device.address(
                            allocations[view.allocation],
                            view.offset,
                            view.bytes,
                        )?;
                        arguments.extend(specs.iter().map(|spec| spec.bind(address)));
                    }
                    let bytes = program.arguments(&arguments, scalars)?;
                    let buffer = device.allocate(bytes.len())?;
                    device.write(buffer, 0, &bytes)?;
                    let args = device.address(buffer, 0, bytes.len())?;
                    kernel_args.push(KernelArgs {
                        kernel: *kernel,
                        views: views.clone(),
                        scalars: scalars.clone(),
                        buffer,
                    });
                    let mut commands = ProgramImage::memory_barrier(dummy);
                    commands.extend(program.dispatch(&Dispatch {
                        program: program_addresses[*kernel],
                        stack,
                        border,
                        dummy,
                        args,
                        global: *global,
                        local: *local,
                    })?);
                    command_words = command_words
                        .checked_add(commands.len())
                        .ok_or(Error::Limit("QCOM frame command words"))?;
                    if command_words > 16 * 1024 * 1024 {
                        return Err(Error::Limit("QCOM frame command words"));
                    }
                    calls.push(Step::Gpu(commands));
                }
            }
        }
        let steps = batch(&calls);
        let entries = bundle
            .graph
            .entrypoints
            .iter()
            .map(|entry| batch(&calls[entry.range()]))
            .collect();
        if let Some(words) = steps
            .iter()
            .filter_map(|step| match step {
                Step::Gpu(words) => Some(words.len()),
                _ => None,
            })
            .max()
        {
            device.reserve_commands(words)?;
        }
        let external = vec![None; bundle.graph.allocations.len()];
        Ok(Self {
            bundle,
            device,
            allocations,
            steps,
            entries,
            kernel_args,
            external,
        })
    }

    fn binding<'a>(&'a self, bindings: &'a [Binding], name: &str) -> Result<&'a View, Error> {
        let binding = bindings
            .iter()
            .find(|binding| binding.name == name)
            .ok_or_else(|| Error::Binding(name.to_owned()))?;
        Ok(&self.bundle.graph.views[binding.view])
    }

    fn reset_mutable(&mut self) -> Result<(), Error> {
        let zeroes = [0u8; 16384];
        for (allocation, &buffer) in self.bundle.graph.allocations.iter().zip(&self.allocations) {
            if let Some(offset) = allocation.weight_offset {
                self.device.write(
                    buffer,
                    0,
                    &self.bundle.weights[offset..offset + allocation.bytes],
                )?;
                continue;
            }
            for offset in (0..allocation.bytes).step_by(zeroes.len()) {
                self.device.write(
                    buffer,
                    offset,
                    &zeroes[..(allocation.bytes - offset).min(zeroes.len())],
                )?;
            }
        }
        Ok(())
    }

    /// Rewrite the argument blocks of every kernel that reads `allocation`.
    fn rebind(&mut self, allocation: usize, address: Option<u64>) -> Result<(), Error> {
        if self.external[allocation] == address {
            return Ok(());
        }
        let graph = &self.bundle.graph;
        for call in &self.kernel_args {
            if !call.views.iter().any(|&v| graph.views[v].allocation == allocation) {
                continue;
            }
            let mut arguments = Vec::new();
            for (&view, specs) in call.views.iter().zip(&graph.kernels[call.kernel].arguments) {
                let view = &graph.views[view];
                let base = match (view.allocation == allocation, address) {
                    (true, Some(external)) => external
                        .checked_add(view.offset as u64)
                        .ok_or(Error::Limit("QCOM address"))?,
                    _ => self.device.address(
                        self.allocations[view.allocation],
                        view.offset,
                        view.bytes,
                    )?,
                };
                arguments.extend(specs.iter().map(|spec| spec.bind(base)));
            }
            let bytes = self.bundle.programs[call.kernel].arguments(&arguments, &call.scalars)?;
            self.device.write(call.buffer, 0, &bytes)?;
        }
        self.external[allocation] = address;
        Ok(())
    }

    fn bind_input(&mut self, name: &str, address: u64, len: usize) -> Result<(), Error> {
        let view = self.binding(&self.bundle.graph.inputs, name)?;
        let allocation = view.allocation;
        // Only a whole, kernel-only input: a host-side copy step or another
        // view at an offset would still use the internal buffer.
        if view.offset != 0
            || len != view.bytes
            || self.bundle.graph.allocations[allocation].bytes != view.bytes
            || self.bundle.graph.allocations[allocation].weight_offset.is_some()
            || self.bundle.graph.calls.iter().any(|call| matches!(call,
                Call::Copy { source, destination }
                    if self.bundle.graph.views[*source].allocation == allocation
                        || self.bundle.graph.views[*destination].allocation == allocation))
        {
            return Err(Error::Contract("QCOM input cannot be bound externally"));
        }
        let address = self.device.external(address, len)?;
        self.rebind(allocation, Some(address))
    }

    fn write_input(&mut self, name: &str, bytes: &[u8]) -> Result<(), Error> {
        let allocation = self.binding(&self.bundle.graph.inputs, name)?.allocation;
        self.rebind(allocation, None)?;
        let view = self.binding(&self.bundle.graph.inputs, name)?;
        if bytes.len() != view.bytes {
            return Err(Error::Size {
                expected: view.bytes,
                actual: bytes.len(),
            });
        }
        self.device
            .write(self.allocations[view.allocation], view.offset, bytes)
    }

    fn read_output(&self, name: &str) -> Result<&[u8], Error> {
        let view = self.binding(&self.bundle.graph.outputs, name)?;
        self.device
            .read(self.allocations[view.allocation], view.offset, view.bytes)
    }

    fn run(&mut self) -> Result<(), Error> {
        self.execute(None)
    }

    fn run_entry(&mut self, name: &str) -> Result<(), Error> {
        let index = entrypoint::find(&self.bundle.graph.entrypoints, name)?;
        self.execute(Some(index))
    }

    fn execute(&mut self, entry: Option<usize>) -> Result<(), Error> {
        let steps = entry.map_or(&self.steps, |index| &self.entries[index]);
        for step in steps {
            match step {
                Step::Gpu(commands) => self.device.execute(commands)?,
                Step::Copy {
                    source,
                    destination,
                } => {
                    let source = &self.bundle.graph.views[*source];
                    let destination = &self.bundle.graph.views[*destination];
                    self.device.copy(
                        self.allocations[source.allocation],
                        source.offset,
                        self.allocations[destination.allocation],
                        destination.offset,
                        source.bytes,
                    )?;
                }
            }
        }
        Ok(())
    }
}

fn batch(calls: &[Step]) -> Vec<Step> {
    let mut steps = Vec::new();
    let mut commands = Vec::new();
    for call in calls {
        match call {
            Step::Gpu(words) => commands.extend(words),
            Step::Copy {
                source,
                destination,
            } => {
                if !commands.is_empty() {
                    steps.push(Step::Gpu(std::mem::take(&mut commands)));
                }
                steps.push(Step::Copy {
                    source: *source,
                    destination: *destination,
                });
            }
        }
    }
    if !commands.is_empty() {
        steps.push(Step::Gpu(commands));
    }
    steps
}
