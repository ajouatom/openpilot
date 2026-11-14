use crate::Error;
use std::io;

#[cfg(all(
    feature = "native-skip-miri",
    target_os = "linux",
    target_pointer_width = "64"
))]
mod abi;
#[cfg(all(
    feature = "native-skip-miri",
    target_os = "linux",
    target_pointer_width = "64"
))]
mod linux;
#[cfg(all(
    feature = "native-skip-miri",
    target_os = "linux",
    target_pointer_width = "64"
))]
pub(super) use linux::Linux;

pub(super) trait Memory {
    fn address(&self) -> u64;
    fn bytes(&self) -> &[u8];
    fn bytes_mut(&mut self) -> &mut [u8];
}

pub(super) trait Driver {
    type Memory: Memory;
    fn allocate(&mut self, size: usize) -> io::Result<Self::Memory>;
    fn submit(&mut self, address: u64, size: usize) -> io::Result<u32>;
    fn wait(&mut self, timestamp: u32) -> io::Result<()>;
    fn close(&mut self) -> io::Result<()>;
    /// GPU address of caller-owned CPU memory (tinygrad `_gpu_map`).
    fn map_user(&mut self, _address: u64, _size: usize) -> io::Result<u64> {
        Err(io::Error::from(io::ErrorKind::Unsupported))
    }
    fn unmap_user(&mut self, _gpu: u64) -> io::Result<()> {
        Ok(())
    }
}

pub(super) struct Device<D: Driver> {
    driver: D,
    allocations: Vec<D::Memory>,
    command_buffer: Option<usize>,
    allocated_bytes: u64,
    closed: bool,
    /// `(host address, size, GPU address)` of mapped external memory.
    externals: Vec<(u64, usize, u64)>,
}

impl<D: Driver> Device<D> {
    pub(super) fn new(driver: D) -> Self {
        Self {
            driver,
            allocations: Vec::new(),
            command_buffer: None,
            allocated_bytes: 0,
            closed: false,
            externals: Vec::new(),
        }
    }

    fn active(&self) -> Result<(), Error> {
        if self.closed {
            Err(Error::Contract("QCOM context is closed after a GPU error"))
        } else {
            Ok(())
        }
    }

    pub(super) fn allocate(&mut self, size: usize) -> Result<usize, Error> {
        self.active()?;
        let total = self
            .allocated_bytes
            .checked_add(size as u64)
            .ok_or(Error::Limit("QCOM allocation sum"))?;
        if size == 0 || total > 4 * 1024 * 1024 * 1024 || self.allocations.len() >= 65536 {
            return Err(Error::Limit("QCOM allocation"));
        }
        let mut memory = self.driver.allocate(size)?;
        if memory.bytes().len() < size {
            return Err(Error::Contract("QCOM mapping is shorter than requested"));
        }
        memory.bytes_mut().fill(0);
        self.allocations.push(memory);
        self.allocated_bytes = total;
        Ok(self.allocations.len() - 1)
    }

    /// GPU address of external CPU memory, mapped once per range. The caller
    /// keeps the memory mapped for this device's lifetime (VisionIPC buffers
    /// live until the camera generation ends, which restarts the process).
    pub(super) fn external(&mut self, host: u64, size: usize) -> Result<u64, Error> {
        self.active()?;
        if host == 0 || size == 0 {
            return Err(Error::Contract("QCOM external range"));
        }
        if let Some(&(_, _, gpu)) = self
            .externals
            .iter()
            .find(|(address, length, _)| *address == host && *length == size)
        {
            return Ok(gpu);
        }
        if self.externals.len() >= 64 {
            for (_, _, gpu) in self.externals.drain(..) {
                if let Err(error) = self.driver.unmap_user(gpu) {
                    eprintln!("QCOM external unmap failed: {error}");
                }
            }
        }
        let aligned = host & !0xfff;
        let length = (size + (host - aligned) as usize + 0xfff) & !0xfff;
        let gpu = self.driver.map_user(aligned, length)? + (host - aligned);
        self.externals.push((host, size, gpu));
        Ok(gpu)
    }

    pub(super) fn address(&self, buffer: usize, offset: usize, size: usize) -> Result<u64, Error> {
        self.read(buffer, offset, size)?;
        self.allocations[buffer]
            .address()
            .checked_add(offset as u64)
            .ok_or(Error::Limit("QCOM address"))
    }

    pub(super) fn read(&self, buffer: usize, offset: usize, size: usize) -> Result<&[u8], Error> {
        self.active()?;
        let memory = self
            .allocations
            .get(buffer)
            .ok_or(Error::Contract("QCOM buffer index"))?;
        let end = offset
            .checked_add(size)
            .ok_or(Error::Limit("QCOM buffer range"))?;
        memory
            .bytes()
            .get(offset..end)
            .ok_or(Error::Contract("QCOM buffer range"))
    }

    pub(super) fn write(
        &mut self,
        buffer: usize,
        offset: usize,
        bytes: &[u8],
    ) -> Result<(), Error> {
        self.active()?;
        let memory = self
            .allocations
            .get_mut(buffer)
            .ok_or(Error::Contract("QCOM buffer index"))?;
        let end = offset
            .checked_add(bytes.len())
            .ok_or(Error::Limit("QCOM buffer range"))?;
        memory
            .bytes_mut()
            .get_mut(offset..end)
            .ok_or(Error::Contract("QCOM buffer range"))?
            .copy_from_slice(bytes);
        Ok(())
    }

    pub(super) fn execute(&mut self, commands: &[u32]) -> Result<(), Error> {
        self.active()?;
        if commands.is_empty() || commands.len() > 16 * 1024 * 1024 {
            return Err(Error::Limit("QCOM command buffer"));
        }
        let size = commands.len() * 4;
        self.reserve_commands(commands.len())?;
        let buffer = self
            .command_buffer
            .ok_or(Error::Contract("QCOM command allocation"))?;
        for (index, word) in commands.iter().enumerate() {
            self.write(buffer, index * 4, &word.to_le_bytes())?;
        }
        let pointer = self.address(buffer, 0, size)?;
        let result = self
            .driver
            .submit(pointer, size)
            .and_then(|timestamp| self.driver.wait(timestamp));
        if let Err(execution) = result {
            return match self.shutdown() {
                Ok(()) => Err(Error::Io(execution)),
                Err(cleanup) => Err(Error::GpuCleanup { execution, cleanup }),
            };
        }
        Ok(())
    }

    pub(super) fn reserve_commands(&mut self, words: usize) -> Result<(), Error> {
        if words == 0 || words > 16 * 1024 * 1024 {
            return Err(Error::Limit("QCOM command buffer"));
        }
        let size = words * 4;
        let buffer = match self.command_buffer {
            Some(buffer) if self.allocations[buffer].bytes().len() >= size => buffer,
            _ => {
                let buffer = self.allocate(size)?;
                self.command_buffer = Some(buffer);
                buffer
            }
        };
        self.command_buffer = Some(buffer);
        Ok(())
    }

    fn shutdown(&mut self) -> io::Result<()> {
        if self.closed {
            return Ok(());
        }
        self.closed = true;
        self.driver.close()
    }

    pub(super) fn copy(
        &mut self,
        source: usize,
        source_offset: usize,
        destination: usize,
        destination_offset: usize,
        size: usize,
    ) -> Result<(), Error> {
        self.read(source, source_offset, size)?;
        self.read(destination, destination_offset, size)?;
        if source == destination {
            self.allocations[source]
                .bytes_mut()
                .copy_within(source_offset..source_offset + size, destination_offset);
        } else {
            let (source_memory, destination_memory) = if source < destination {
                let (before, after) = self.allocations.split_at_mut(destination);
                (&before[source], &mut after[0])
            } else {
                let (before, after) = self.allocations.split_at_mut(source);
                (&after[0], &mut before[destination])
            };
            destination_memory.bytes_mut()[destination_offset..destination_offset + size]
                .copy_from_slice(&source_memory.bytes()[source_offset..source_offset + size]);
        }
        Ok(())
    }
}

impl<D: Driver> Drop for Device<D> {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown() {
            eprintln!("QCOM context cleanup failed: {error}");
        }
    }
}

#[cfg(test)]
mod tests;
