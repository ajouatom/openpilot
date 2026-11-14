//! SCHED_FIFO for the updater (formerly `updater_scheduling.cc`).
pub(super) mod ffi {
    pub struct Result {
        pub error: i32,
    }

    pub fn set_fifo(priority: i32) -> Result {
        let param = libc::sched_param { sched_priority: priority };
        // SAFETY: `param` is a live, initialized struct for this call only.
        let result = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
        Result {
            error: if result < 0 {
                std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
            } else {
                0
            },
        }
    }
}
