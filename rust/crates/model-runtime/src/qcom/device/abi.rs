#[repr(C)]
#[derive(Default)]
pub(super) struct DevInfo {
    pub device_id: u32,
    pub chip_id: u32,
    pub mmu_enabled: u32,
    pub padding0: u32,
    pub gmem_gpubaseaddr: u64,
    pub gpu_id: u32,
    pub padding1: u32,
    pub gmem_sizebytes: u64,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct Property {
    pub kind: u32,
    pub padding: u32,
    pub value: u64,
    pub sizebytes: u64,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct WaitTimestamp {
    pub context_id: u32,
    pub timestamp: u32,
    pub timeout: u32,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct CreateContext {
    pub flags: u32,
    pub drawctxt_id: u32,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct DestroyContext {
    pub drawctxt_id: u32,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct Allocate {
    pub size: u64,
    pub flags: u64,
    pub va_len: u64,
    pub mmapsize: u64,
    pub id: u32,
    pub metadata_len: u32,
    pub metadata: u64,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct Free {
    pub flags: u64,
    pub private: u64,
    pub id: u32,
    pub kind: u32,
    pub len: u32,
    pub padding: u32,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct CommandObject {
    pub offset: u64,
    pub gpuaddr: u64,
    pub size: u64,
    pub flags: u32,
    pub id: u32,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct GpuCommand {
    pub flags: u64,
    pub cmdlist: u64,
    pub cmdsize: u32,
    pub numcmds: u32,
    pub objlist: u64,
    pub objsize: u32,
    pub numobjs: u32,
    pub synclist: u64,
    pub syncsize: u32,
    pub numsyncs: u32,
    pub context_id: u32,
    pub timestamp: u32,
}

#[repr(C)]
#[derive(Default)]
pub(super) struct PowerConstraint {
    pub kind: u32,
    pub context_id: u32,
    pub data: u64,
    pub size: u64,
}

/// `struct kgsl_map_user_mem`.
#[repr(C)]
#[derive(Default)]
pub(super) struct MapUserMem {
    pub fd: i32,
    pub padding: u32,
    pub gpuaddr: u64,
    pub len: u64,
    pub offset: u64,
    pub hostptr: u64,
    pub memtype: u32,
    pub flags: u32,
}

/// `struct kgsl_sharedmem_free`.
#[repr(C)]
#[derive(Default)]
pub(super) struct SharedmemFree {
    pub gpuaddr: u64,
}

/// `KGSL_USER_MEM_TYPE_ADDR`.
pub(super) const USER_MEM_TYPE_ADDR: u32 = 2;

pub(super) trait Payload {}
impl Payload for MapUserMem {}
impl Payload for SharedmemFree {}
impl Payload for Property {}
impl Payload for WaitTimestamp {}
impl Payload for CreateContext {}
impl Payload for DestroyContext {}
impl Payload for Allocate {}
impl Payload for Free {}
impl Payload for GpuCommand {}

const fn request<T>(direction: u32, number: u32) -> libc::c_ulong {
    ((direction << 30) | ((std::mem::size_of::<T>() as u32) << 16) | (9 << 8) | number)
        as libc::c_ulong
}

pub(super) const GET_PROPERTY: libc::c_ulong = request::<Property>(3, 0x02);
pub(super) const WAIT_TIMESTAMP: libc::c_ulong = request::<WaitTimestamp>(1, 0x07);
pub(super) const CREATE_CONTEXT: libc::c_ulong = request::<CreateContext>(3, 0x13);
pub(super) const DESTROY_CONTEXT: libc::c_ulong = request::<DestroyContext>(1, 0x14);
pub(super) const SET_PROPERTY: libc::c_ulong = request::<Property>(1, 0x32);
pub(super) const ALLOCATE: libc::c_ulong = request::<Allocate>(3, 0x45);
pub(super) const FREE: libc::c_ulong = request::<Free>(1, 0x46);
pub(super) const MAP_USER_MEM: libc::c_ulong = request::<MapUserMem>(3, 0x15);
pub(super) const SHAREDMEM_FREE: libc::c_ulong = request::<SharedmemFree>(1, 0x21);
pub(super) const GPU_COMMAND: libc::c_ulong = request::<GpuCommand>(3, 0x4a);
