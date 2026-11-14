//! Scheduler syscalls (formerly the C++ shim `native/scheduling.cc`).
pub(super) mod ffi {
    pub struct SchedulerResult {
        pub value: i32,
        pub error: i32,
    }

    fn errno() -> i32 {
        std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
    }

    pub fn get_policy(tid: i32) -> SchedulerResult {
        // SAFETY: scalar syscall; retains no memory.
        let value = unsafe { libc::sched_getscheduler(tid) };
        SchedulerResult { value, error: if value < 0 { errno() } else { 0 } }
    }

    pub fn set_other(tid: i32) -> SchedulerResult {
        let param = libc::sched_param { sched_priority: 0 };
        // SAFETY: `param` is a live, initialized struct for this call only.
        let value = unsafe { libc::sched_setscheduler(tid, libc::SCHED_OTHER, &param) };
        SchedulerResult { value, error: if value < 0 { errno() } else { 0 } }
    }
}
