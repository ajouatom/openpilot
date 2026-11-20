//! Source terminal exception grammar. Mentions earlier in a traceback never qualify.
// Source 8864280c verifies identical latched +/-1 USB meanings for these two
// isolated Tinygrad runtimes only. Unknown runtimes remain numerical failures.
const VERIFIED_USB_RUNTIMES: [&str; 2] = [
    "90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2",
    "c7c58622652231359ecea1a4253b63e157d002242bf596ede1b853f941f937ac",
];

fn terminal(detail: &str) -> &str {
    detail.trim_end().rsplit('\n').next().unwrap_or("")
}
fn runtime_terminal(detail: &str) -> &str {
    terminal(detail)
        .strip_prefix("RuntimeError: ")
        .unwrap_or(terminal(detail))
}
fn decimal(value: &str) -> bool {
    !value.is_empty() && value.bytes().all(|v| v.is_ascii_digit())
}

#[must_use]
pub fn gpu_bootloader_timeout(detail: &str, phase: &str) -> bool {
    if !matches!(phase, "load" | "boot_validation") {
        return false;
    }
    let Some(tail) = terminal(detail).strip_prefix("TimeoutError: BL not ready. Timed out after ")
    else {
        return false;
    };
    let Some((ms, condition)) = tail.split_once(" ms, condition not met: ") else {
        return false;
    };
    let Some((actual, expected)) = condition.split_once(" != ") else {
        return false;
    };
    decimal(ms) && decimal(actual) && decimal(expected)
}

/// The GPU stopped answering PCIe requests through the USB bridge: tinygrad
/// reports Unsupported Request as "target might not be reachable". On the
/// IONIQ 9 (comma four) VRAM and MMIO reads both failed right after 640 MiB
/// of weights were written, i.e. the device dropped off, not a model defect.
/// Retried later instead of quarantining the model.
#[must_use]
pub fn gpu_unreachable(detail: &str) -> bool {
    let line = runtime_terminal(detail);
    line.starts_with("TLP completion status: Unsupported Request")
        || line.starts_with("TLP completion status: Completer Abort")
        || line.starts_with("TLP error after retries")
}

#[must_use]
pub fn gpu_worker_busy(detail: &str) -> bool {
    runtime_terminal(detail) == "previous eGPU worker is still exiting"
}

#[must_use]
pub fn gpu_transport_failure(detail: &str) -> bool {
    let lower = terminal(detail).to_ascii_lowercase();
    let line = lower.strip_prefix("runtimeerror: ").unwrap_or(&lower);
    let Some((operation, error)) = line.split_once(": ") else {
        return false;
    };
    let operation_matches = matches!(
        operation,
        "libusb_control_transfer"
            | "libusb_bulk_transfer"
            | "libusb_interrupt_transfer"
            | "libusb_open"
            | "libusb_claim_interface"
    ) || operation
        .strip_prefix("bulk out 0x")
        .or_else(|| operation.strip_prefix("bulk in 0x"))
        .and_then(|tail| tail.strip_suffix(" failed"))
        .is_some_and(|address| {
            !address.is_empty() && address.bytes().all(|v| v.is_ascii_hexdigit())
        });
    operation_matches
        && matches!(
            error,
            "no such device (it may have been disconnected)"
                | "input/output error"
                | "operation timed out"
                | "pipe error"
                | "resource busy"
                | "system call interrupted"
                | "system call interrupted (perhaps due to signal)"
        )
}

#[must_use]
pub fn gpu_usb_link_failure(detail: &str, runtime_sha256: &str) -> bool {
    if !VERIFIED_USB_RUNTIMES.contains(&runtime_sha256) {
        return false;
    }
    let Some((device, code)) = runtime_terminal(detail).split_once(" failed with ") else {
        return false;
    };
    matches!(code, "-1" | "1")
        && (device == "AMD" || device.strip_prefix("AMD:").is_some_and(decimal))
}

#[cfg(test)]
mod tests {
    use super::gpu_unreachable;

    #[test]
    fn an_unreachable_gpu_is_not_a_model_defect() {
        // Worker output from the IONIQ 9 (comma four, Light Speed 2), 2026-11-20.
        let detail = "eGPU worker startup: bind_model\neGPU worker startup: prepare_runtime\n\
            usbgpu AMD finalization failed: TLP completion status: Unsupported Request: 0x10004b80\n\
            TLP completion status: Unsupported Request: 0x80298000c\n";
        assert!(gpu_unreachable(detail));
        assert!(gpu_unreachable(
            "TLP error after retries: ret_status=1, address=0x10000000"
        ));
        // Only the terminal line counts; a numerical failure stays a model failure.
        assert!(!gpu_unreachable(
            "TLP completion status: Unsupported Request: 0x1\nnon-finite model output"
        ));
    }
}
