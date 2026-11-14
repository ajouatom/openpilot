use super::{abi, Driver, Memory};
use std::{
    fs::{File, OpenOptions},
    io,
    mem::size_of,
    os::fd::{AsRawFd, IntoRawFd},
    ptr::NonNull,
};

pub(in crate::qcom) struct Linux {
    file: Option<File>,
    context: Option<u32>,
}

pub(in crate::qcom) struct Mapping {
    pointer: NonNull<u8>,
    size: usize,
    mapped_size: usize,
}

impl Memory for Mapping {
    fn address(&self) -> u64 {
        self.pointer.as_ptr() as u64
    }
    fn bytes(&self) -> &[u8] {
        // SAFETY: the mapping owns this range; Device exposes CPU access only after a successful GPU wait.
        unsafe { std::slice::from_raw_parts(self.pointer.as_ptr(), self.size) }
    }
    fn bytes_mut(&mut self) -> &mut [u8] {
        // SAFETY: exclusive Device access and synchronous submission prevent concurrent CPU/GPU access.
        unsafe { std::slice::from_raw_parts_mut(self.pointer.as_ptr(), self.size) }
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: Device releases the KGSL fd before mappings; KGSL owns pending GPU references to these pages.
        if unsafe { libc::munmap(self.pointer.as_ptr().cast(), self.mapped_size) } != 0 {
            eprintln!("QCOM munmap failed: {}", io::Error::last_os_error());
        }
    }
}

impl Linux {
    pub(in crate::qcom) fn open(priority: u8) -> io::Result<Self> {
        if priority > 15 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "QCOM priority exceeds 15",
            ));
        }
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/kgsl-3d0")?;
        let mut driver = Self {
            file: Some(file),
            context: None,
        };
        let mut info = abi::DevInfo::default();
        let mut property = abi::Property {
            kind: 1,
            value: (&mut info as *mut abi::DevInfo) as u64,
            sizebytes: size_of::<abi::DevInfo>() as u64,
            ..Default::default()
        };
        driver.ioctl(abi::GET_PROPERTY, &mut property)?;
        if (info.chip_id >> 8) != 0x060300 {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!("QCOMCL a630 required, chip_id={:#x}", info.chip_id),
            ));
        }
        let mut context = abi::CreateContext {
            flags: 0x10 | 0x800 | 0x200 | 2 | (u32::from(priority) << 12) | (2 << 25),
            ..Default::default()
        };
        driver.ioctl(abi::CREATE_CONTEXT, &mut context)?;
        driver.context = Some(context.drawctxt_id);
        let mut level = 1_u32;
        let mut constraint = abi::PowerConstraint {
            kind: 1,
            context_id: context.drawctxt_id,
            data: (&mut level as *mut u32) as u64,
            size: size_of::<u32>() as u64,
        };
        let mut property = abi::Property {
            kind: 0x12,
            value: (&mut constraint as *mut abi::PowerConstraint) as u64,
            sizebytes: size_of::<abi::PowerConstraint>() as u64,
            ..Default::default()
        };
        driver.ioctl(abi::SET_PROPERTY, &mut property)?;
        Ok(driver)
    }

    fn file(&self) -> io::Result<&File> {
        self.file
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::BrokenPipe, "KGSL device closed"))
    }

    fn ioctl<T: abi::Payload>(&self, request: libc::c_ulong, payload: &mut T) -> io::Result<()> {
        if ((request >> 16) & 0x3fff) as usize != size_of::<T>() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "KGSL ioctl payload size",
            ));
        }
        // SAFETY: Payload is private and implemented only for integer-only repr(C) ABI structs; nested pointers live through this call.
        let result =
            unsafe { libc::ioctl(self.file()?.as_raw_fd(), request as _, payload as *mut T) };
        if result < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    fn allocation_error(&self, id: u32, error: io::Error) -> io::Error {
        match self.ioctl(
            abi::FREE,
            &mut abi::Free {
                id,
                ..Default::default()
            },
        ) {
            Ok(()) => error,
            Err(cleanup) => io::Error::other(format!(
                "{error}; KGSL allocation cleanup failed: {cleanup}"
            )),
        }
    }
}

impl Driver for Linux {
    type Memory = Mapping;

    fn allocate(&mut self, size: usize) -> io::Result<Mapping> {
        let mapped_size = size
            .checked_add(4095)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "KGSL size overflow"))?
            & !4095;
        let mut allocation = abi::Allocate {
            size: mapped_size as u64,
            mmapsize: mapped_size as u64,
            flags: (12 << 16) | 0x10000000,
            ..Default::default()
        };
        self.ioctl(abi::ALLOCATE, &mut allocation)?;
        if allocation.mmapsize != mapped_size as u64 {
            return Err(self.allocation_error(
                allocation.id,
                io::Error::new(io::ErrorKind::InvalidData, "unexpected KGSL mmap length"),
            ));
        }
        // SAFETY: the kernel allocated this id for the live fd; the mapping length and page-aligned offset use its KGSL ABI.
        let pointer = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                mapped_size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                self.file()?.as_raw_fd(),
                i64::from(allocation.id) * 4096,
            )
        };
        if pointer == libc::MAP_FAILED {
            let error = io::Error::last_os_error();
            return Err(self.allocation_error(allocation.id, error));
        }
        let Some(pointer) = NonNull::new(pointer.cast()) else {
            // SAFETY: mmap succeeded at address zero; release that exact range rather than constructing a null Rust slice.
            if unsafe { libc::munmap(pointer, mapped_size) } != 0 {
                eprintln!(
                    "QCOM null mapping cleanup failed: {}",
                    io::Error::last_os_error()
                );
            }
            return Err(self.allocation_error(
                allocation.id,
                io::Error::other("KGSL mmap returned address zero"),
            ));
        };
        Ok(Mapping {
            pointer,
            size,
            mapped_size,
        })
    }

    fn map_user(&mut self, address: u64, size: usize) -> io::Result<u64> {
        let mut map = abi::MapUserMem {
            len: size as u64,
            hostptr: address,
            memtype: abi::USER_MEM_TYPE_ADDR,
            ..Default::default()
        };
        self.ioctl(abi::MAP_USER_MEM, &mut map)?;
        if map.gpuaddr == 0 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "KGSL user map address"));
        }
        Ok(map.gpuaddr)
    }

    fn unmap_user(&mut self, gpu: u64) -> io::Result<()> {
        self.ioctl(abi::SHAREDMEM_FREE, &mut abi::SharedmemFree { gpuaddr: gpu & !0xfff })
    }

    fn submit(&mut self, address: u64, size: usize) -> io::Result<u32> {
        let mut object = abi::CommandObject {
            gpuaddr: address,
            size: size as u64,
            flags: 1,
            ..Default::default()
        };
        let mut command = abi::GpuCommand {
            cmdlist: (&mut object as *mut abi::CommandObject) as u64,
            numcmds: 1,
            cmdsize: size_of::<abi::CommandObject>() as u32,
            context_id: self
                .context
                .ok_or_else(|| io::Error::new(io::ErrorKind::BrokenPipe, "KGSL context closed"))?,
            ..Default::default()
        };
        self.ioctl(abi::GPU_COMMAND, &mut command)?;
        Ok(command.timestamp)
    }

    fn wait(&mut self, timestamp: u32) -> io::Result<()> {
        let mut wait = abi::WaitTimestamp {
            context_id: self
                .context
                .ok_or_else(|| io::Error::new(io::ErrorKind::BrokenPipe, "KGSL context closed"))?,
            timestamp,
            timeout: u32::MAX,
        };
        loop {
            match self.ioctl(abi::WAIT_TIMESTAMP, &mut wait) {
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                result => return result,
            }
        }
    }

    fn close(&mut self) -> io::Result<()> {
        let destroy = if let Some(drawctxt_id) = self.context.take() {
            self.ioctl(
                abi::DESTROY_CONTEXT,
                &mut abi::DestroyContext { drawctxt_id },
            )
        } else {
            Ok(())
        };
        let close = if let Some(file) = self.file.take() {
            // SAFETY: into_raw_fd transfers sole ownership; Linux consumes the fd even when close reports an error. Never retry it.
            if unsafe { libc::close(file.into_raw_fd()) } == 0 {
                Ok(())
            } else {
                Err(io::Error::last_os_error())
            }
        } else {
            Ok(())
        };
        match (destroy, close) {
            (Err(destroy), Err(close)) => Err(io::Error::other(format!(
                "KGSL context destroy failed: {destroy}; fd close failed: {close}"
            ))),
            (Err(error), _) | (_, Err(error)) => Err(error),
            _ => Ok(()),
        }
    }
}

impl Drop for Linux {
    fn drop(&mut self) {
        if let Err(error) = self.close() {
            eprintln!("QCOM device cleanup failed: {error}");
        }
    }
}
