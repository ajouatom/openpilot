//! carrot's `navigation/preview.py`: one small tmpfs PNG of the route map
//! shared by Carrot Web and the cluster HUD (never model input), with the
//! reason it shows what it shows.
use std::{io::Write, path::Path};

pub const PREVIEW_PATH: &str = "/dev/shm/carrot_nav_map.png";
pub const STATUS_PATH: &str = "/dev/shm/carrot_nav_status.json";
/// Carrot Web touches this while a page shows the map.
pub const WEB_DEMAND_PATH: &str = "/dev/shm/carrot_nav_web";
pub const PREVIEW_MAX_AGE: f64 = 2.;

/// `preview_fresh`: modified within 2 s by the wall clock.
pub fn fresh(path: &Path) -> bool {
    std::fs::metadata(path)
        .and_then(|meta| meta.modified())
        .ok()
        .and_then(|modified| std::time::SystemTime::now().duration_since(modified).ok())
        .is_some_and(|age| age.as_secs_f64() < PREVIEW_MAX_AGE)
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend((data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend(crc.to_be_bytes());
}

/// An 8-bit grayscale PNG (fast compression, as PIL's `compress_level=1`).
pub fn encode_gray(image: &[u8], width: u32, height: u32) -> std::io::Result<Vec<u8>> {
    if image.len() != (width * height) as usize {
        return Err(std::io::Error::other("preview size"));
    }
    let mut raw = Vec::with_capacity(image.len() + height as usize);
    for row in image.chunks_exact(width as usize) {
        raw.push(0); // filter: none
        raw.extend_from_slice(row);
    }
    let mut encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::fast());
    encoder.write_all(&raw)?;
    let data = encoder.finish()?;
    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut header = Vec::with_capacity(13);
    header.extend(width.to_be_bytes());
    header.extend(height.to_be_bytes());
    header.extend([8, 0, 0, 0, 0]); // 8-bit, grayscale, deflate, no filter, no interlace
    chunk(&mut out, b"IHDR", &header);
    chunk(&mut out, b"IDAT", &data);
    chunk(&mut out, b"IEND", &[]);
    Ok(out)
}

fn replace(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let temporary = path.with_extension("tmp");
    std::fs::write(&temporary, bytes)?;
    std::fs::rename(&temporary, path)
}

/// `write_preview`: the image, then its reason, each replaced atomically.
pub fn write(
    image: &[u8],
    width: u32,
    height: u32,
    reason: &str,
    dir: &Path,
) -> std::io::Result<()> {
    let png = encode_gray(image, width, height)?;
    replace(&dir.join("carrot_nav_map.png"), &png)?;
    let status = serde_json::json!({"reason": reason}).to_string();
    replace(&dir.join("carrot_nav_status.json"), status.as_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_the_png_reference() {
        // CRC of "IEND" with no data, as every PNG ends.
        assert_eq!(crc32(b"IEND"), 0xae42_6082);
    }

    #[test]
    fn png_has_the_signature_header_and_round_trips_through_inflate() {
        let image: Vec<u8> = (0..16u8).collect();
        let png = encode_gray(&image, 4, 4).unwrap();
        assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
        assert_eq!(&png[12..16], b"IHDR");
        assert_eq!(u32::from_be_bytes(png[16..20].try_into().unwrap()), 4);
        assert_eq!(&png[png.len() - 8..png.len() - 4], b"IEND");
        let length = u32::from_be_bytes(png[33..37].try_into().unwrap()) as usize;
        assert_eq!(&png[37..41], b"IDAT");
        let mut inflated = Vec::new();
        std::io::Read::read_to_end(
            &mut flate2::read::ZlibDecoder::new(&png[41..41 + length]),
            &mut inflated,
        )
        .unwrap();
        assert_eq!(inflated.len(), 4 * 5);
        assert_eq!(&inflated[1..5], &[0, 1, 2, 3]);
        assert!(encode_gray(&image, 5, 4).is_err());
    }

    #[test]
    fn write_replaces_the_image_and_the_reason() {
        let dir = std::env::temp_dir().join(format!("mapsd-preview-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        write(&[0; 4], 2, 2, "route", &dir).unwrap();
        let status = std::fs::read_to_string(dir.join("carrot_nav_status.json")).unwrap();
        assert_eq!(status, r#"{"reason":"route"}"#);
        assert!(fresh(&dir.join("carrot_nav_map.png")));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
