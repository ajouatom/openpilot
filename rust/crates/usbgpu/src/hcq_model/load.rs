use super::{
    manifest::{Buffer, Manifest, Patch, Space},
    storage::{self, Bound},
    Device, Model, Request,
};
use crate::{
    hcq_vm::{Memory, Program},
    Error,
};
use std::{fs::File, path::Path};

impl<D: Device> Model<D> {
    pub fn load(descriptor: &[u8], path: &Path, mut device: D) -> Result<Self, Error> {
        let lock = device.bus_lock();
        let _guard = lock.as_ref().map(|lock| lock.enter()).transpose()?;
        let manifest = Manifest::parse(descriptor)?;
        let program = Program::parse(&serde_json::to_vec(&manifest.dispatcher)?)?;
        let mut file = File::open(path)?;
        storage::verify(&mut file, manifest.model_bytes, &manifest.model_sha256)?;
        crate::worker_progress::load_stage("load_model");
        let mut uploaded = 0u64;
        let mut next_report = 128 << 20;
        let mut memory = Memory::default();
        let mut buffers = Vec::with_capacity(manifest.buffers.len());
        let mut total = 0u64;
        for spec in &manifest.buffers {
            let (request, initial, cpu) = match spec {
                Buffer::Allocation {
                    bytes,
                    host,
                    cpu_access,
                    uncached,
                    initial,
                } => (
                    Request {
                        bytes: *bytes,
                        host: *host,
                        cpu_access: *cpu_access,
                        uncached: *uncached,
                        tag: None,
                        elements: 0,
                    },
                    Some(initial),
                    false,
                ),
                Buffer::Placeholder {
                    tag,
                    bytes,
                    elements,
                    host,
                    cpu_access,
                    uncached,
                    device,
                } => {
                    if !["CPU", "AMD"].contains(&device.as_str()) {
                        return Err(Error::Contract("HCQ placeholder device"));
                    }
                    (
                        Request {
                            bytes: *bytes,
                            host: *host,
                            cpu_access: *cpu_access,
                            uncached: *uncached,
                            tag: Some(tag),
                            elements: *elements,
                        },
                        None,
                        device == "CPU"
                            || ["usb_host", "put_value_compute_0"].contains(&tag.as_str()),
                    )
                }
            };
            total = total
                .checked_add(request.bytes)
                .ok_or(Error::Contract("HCQ allocation total overflow"))?;
            if total > 4 << 30 {
                return Err(Error::Contract("HCQ allocation total limit"));
            }
            let bound = if cpu {
                let mut bytes = vec![
                    0;
                    usize::try_from(request.bytes)
                        .map_err(|_| Error::Contract("HCQ host allocation size"))?
                ];
                let token = match request.tag {
                    Some("libusb_control_transfer") => 1u64,
                    Some("libusb_bulk_transfer") => 2,
                    Some("usb_host") => 1,
                    _ => 0,
                };
                if token != 0 {
                    bytes
                        .get_mut(..8)
                        .ok_or(Error::Contract("HCQ host special buffer size"))?
                        .copy_from_slice(&token.to_le_bytes());
                }
                Bound::Host {
                    address: memory.allocate(bytes)?,
                    bytes: request.bytes,
                }
            } else {
                Bound::Device(device.allocate(request)?)
            };
            if let Some(blob) = initial {
                storage::load_blob(&mut file, blob, |offset, data| {
                    bound.write(offset, data, &mut memory, &mut device)?;
                    uploaded += data.len() as u64;
                    crate::worker_progress::mark("upload_weights");
                    if uploaded >= next_report {
                        eprintln!("eGPU worker uploaded {} MiB of weights", uploaded >> 20);
                        next_report += 128 << 20;
                    }
                    Ok(())
                })?;
            }
            buffers.push(bound);
        }
        crate::worker_progress::load_stage("bind_model");
        let mut bindings = Vec::with_capacity(manifest.bindings.len());
        for binding in &manifest.bindings {
            let allocation = if let Some(alias) = binding.alias {
                bindings[alias]
            } else {
                let allocation = device.allocate(Request {
                    bytes: binding.bytes,
                    host: false,
                    cpu_access: true,
                    uncached: false,
                    tag: None,
                    elements: 0,
                })?;
                let zeroes = vec![
                    0;
                    usize::try_from(binding.bytes.min(256 << 10))
                        .map_err(|_| Error::Contract("HCQ tensor size overflow"))?
                ];
                let mut offset = 0;
                while offset < binding.bytes {
                    let size = usize::try_from((binding.bytes - offset).min(zeroes.len() as u64))
                        .map_err(|_| Error::Contract("HCQ tensor zero size overflow"))?;
                    device.write(allocation, offset, &zeroes[..size])?;
                    offset += size as u64;
                }
                allocation
            };
            bindings.push(allocation);
        }
        for patch in &manifest.patches {
            match patch {
                Patch::Blob { view, blob } => {
                    let bound = storage::buffer(&buffers, *view)?;
                    storage::load_blob(&mut file, blob, |offset, data| {
                        bound.write(
                            view.offset
                                .checked_add(offset)
                                .ok_or(Error::Contract("HCQ patch offset overflow"))?,
                            data,
                            &mut memory,
                            &mut device,
                        )
                    })?;
                }
                Patch::Word { view, bytes, value } => {
                    if ![1, 2, 4, 8].contains(bytes) {
                        return Err(Error::Contract("HCQ patch scalar width"));
                    }
                    let value = storage::expression(value, &buffers, &bindings)?.to_le_bytes();
                    storage::buffer(&buffers, *view)?.write(
                        view.offset,
                        &value[..usize::from(*bytes)],
                        &mut memory,
                        &mut device,
                    )?;
                }
            }
        }
        let arguments = if let Some(values) = &manifest.argument_values {
            values
                .iter()
                .map(|value| storage::expression(value, &buffers, &bindings))
                .collect::<Result<Vec<_>, _>>()?
        } else {
            manifest
                .arguments
                .iter()
                .map(|view| storage::buffer(&buffers, *view)?.address(view.offset, Space::Host))
                .collect::<Result<Vec<_>, _>>()?
        };
        for parameter in &manifest.parameters {
            if parameter.name.is_empty() {
                return Err(Error::Contract("HCQ parameter name missing"));
            }
            let address = *arguments
                .get(parameter.slot)
                .ok_or(Error::Contract("HCQ parameter missing"))?;
            if parameter.bytes == 0 {
                continue;
            }
            if let Some(values) = &manifest.argument_values {
                if let super::manifest::Expression::Address { view, .. } = &values[parameter.slot] {
                    if let Bound::Device(_) = storage::buffer(&buffers, *view)? {
                        storage::buffer(&buffers, *view)?.address(
                            view.offset
                                .checked_add(parameter.bytes)
                                .ok_or(Error::Contract("HCQ parameter extent overflow"))?,
                            Space::Host,
                        )?;
                        if address == 0 {
                            return Err(Error::Contract("HCQ remote host mapping missing"));
                        }
                        continue;
                    }
                }
            }
            memory.read(
                address,
                usize::try_from(parameter.bytes)
                    .map_err(|_| Error::Contract("HCQ parameter size overflow"))?,
            )?;
        }
        let execution = program.bind(&mut memory, &arguments)?;
        Ok(Self {
            manifest,
            device,
            memory,
            buffers,
            bindings,
            execution,
            failed: false,
        })
    }
}
