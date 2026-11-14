//! `std::stof` semantics (formerly the C++ shim `native/params_numeric.cc`):
//! leading whitespace, a numeric prefix, hex/inf/nan; no conversion or a
//! range error is invalid.
pub(super) mod ffi {
    pub struct FloatResult {
        pub value: f32,
        pub valid: bool,
    }

    pub fn params_float(bytes: &[u8]) -> FloatResult {
        if bytes.is_empty() {
            return FloatResult { value: 0.0, valid: true };
        }
        // std::string keeps embedded NULs, but strtof stops at the first.
        let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        let mut text = bytes[..end].to_vec();
        text.push(0);
        let mut rest: *mut libc::c_char = std::ptr::null_mut();
        // SAFETY: `text` is NUL-terminated and outlives the call; errno is
        // this thread's.
        let value = unsafe {
            *libc::__errno_location() = 0;
            libc::strtof(text.as_ptr().cast(), &mut rest)
        };
        // SAFETY: as above.
        let range = unsafe { *libc::__errno_location() } == libc::ERANGE;
        if rest.cast_const() == text.as_ptr().cast() || range {
            return FloatResult { value: 0.0, valid: false };
        }
        FloatResult { value, valid: true }
    }

    #[cfg(test)]
    mod tests {
        use super::params_float;

        #[test]
        fn matches_stof() {
            for (text, value, valid) in [
                ("", 0.0, true),
                ("1.5", 1.5, true),
                ("  2.25abc", 2.25, true),
                ("-3", -3.0, true),
                ("0x10", 16.0, true),
                ("abc", 0.0, false),
                ("1e50", 0.0, false),
                ("1\05", 1.0, true),
            ] {
                let result = params_float(text.as_bytes());
                assert_eq!((result.value, result.valid), (value, valid), "{text:?}");
            }
            assert!(params_float(b"inf").value.is_infinite());
        }
    }
}
