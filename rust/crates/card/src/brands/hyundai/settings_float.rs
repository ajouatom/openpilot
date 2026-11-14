use super::Error;
use openpilot_params::Params;

/// A stored value that is not a number reads as 0, like a missing one.
/// carrot raised here, and card re-reads its settings every 10 frames, so a
/// single corrupt setting file restarted card (and vehicle control) forever.
pub fn read(params: &Params, key: &str) -> Result<f64, Error> {
    let Some(bytes) = params.get(key)? else {
        return Ok(0.);
    };
    Ok(parse(&bytes, key).unwrap_or(0.))
}

fn parse(bytes: &[u8], key: &str) -> Result<f64, Error> {
    if bytes.is_empty() {
        return Ok(0.);
    }
    let value = std::str::from_utf8(bytes)?.trim_start_matches(|c: char| c.is_ascii_whitespace());
    let mut end = 0;
    let bytes = value.as_bytes();
    if matches!(bytes.first(), Some(b'+' | b'-')) {
        end += 1;
    }
    let start = end;
    for name in ["infinity", "inf", "nan"] {
        if value[start..].to_ascii_lowercase().starts_with(name) {
            let sign = if bytes.first() == Some(&b'-') {
                0x8000_0000
            } else {
                0
            };
            let number = if name == "nan" {
                super::float_number::nan(&value[start + 3..], sign)
            } else {
                f32::from_bits(sign | 0x7f80_0000)
            };
            return Ok(f64::from(number));
        }
    }
    if value
        .get(start..start + 2)
        .is_some_and(|prefix| prefix.eq_ignore_ascii_case("0x"))
    {
        if let Some(number) =
            super::float_number::hex(&value[start + 2..], bytes.first() == Some(&b'-'), key)?
        {
            return Ok(f64::from(number));
        }
    }
    while bytes.get(end).is_some_and(u8::is_ascii_digit) {
        end += 1;
    }
    if bytes.get(end) == Some(&b'.') {
        end += 1;
        while bytes.get(end).is_some_and(u8::is_ascii_digit) {
            end += 1;
        }
    }
    let mantissa_end = end;
    if matches!(bytes.get(end), Some(b'e' | b'E')) {
        end += 1;
        if matches!(bytes.get(end), Some(b'+' | b'-')) {
            end += 1;
        }
        let exponent_start = end;
        while bytes.get(end).is_some_and(u8::is_ascii_digit) {
            end += 1;
        }
        if end == exponent_start {
            end = mantissa_end;
        }
    }
    let result = value[..end].parse::<f32>()?;
    super::float_number::decimal_range(&value[..end], result, key)?;
    Ok(f64::from(result))
}

#[cfg(test)]
mod corrupt_tests {
    #[test]
    fn a_corrupt_setting_reads_as_zero_instead_of_failing_card() {
        let root = std::env::temp_dir().join(format!("card-settings-{}", std::process::id()));
        std::fs::create_dir_all(&root).unwrap();
        let params = openpilot_params::Params::open(&root, "d").unwrap();
        for (raw, int, float) in [("12", 12, 12.), ("1.5", 1, 1.5), ("True", 0, 0.), ("\u{ff}x", 0, 0.), ("", 0, 0.)] {
            params.put("CruiseSpeed2", raw.as_bytes()).unwrap();
            assert_eq!(super::super::parameters::setting_int(&params, "CruiseSpeed2").unwrap(), int, "{raw:?}");
            assert_eq!(super::read(&params, "CruiseSpeed2").unwrap(), float, "{raw:?}");
        }
    }
}
