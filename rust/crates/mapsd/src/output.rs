//! The rendered route map handed to the model link: one shared-memory file,
//! replaced atomically per render. carrot's mapsd served it over VisionIPC.
use std::{fs, io, path::Path};

pub const PATH: &str = "/dev/shm/carrot-nav-map";
/// Touched by modeld while a navigation graph is connected.
pub const DEMAND_PATH: &str = "/dev/shm/carrot-nav-map-demand";
const MAGIC: &[u8; 4] = b"NMAP";
pub const MAP_BYTES: usize = crate::renderer::SIZE * crate::renderer::SIZE;
const HEADER: usize = 4 + 8 + 8 + 1;

#[derive(Clone, Debug, PartialEq)]
pub struct Map {
    pub route_id: u64,
    /// Monotonic ns at render start: a slow tile request never looks fresh.
    pub rendered_ns: u64,
    pub valid: bool,
    pub image: Vec<u8>,
}

pub fn write(path: &Path, map: &Map) -> io::Result<()> {
    let mut bytes = Vec::with_capacity(HEADER + MAP_BYTES);
    bytes.extend_from_slice(MAGIC);
    bytes.extend_from_slice(&map.route_id.to_le_bytes());
    bytes.extend_from_slice(&map.rendered_ns.to_le_bytes());
    bytes.push(u8::from(map.valid));
    bytes.extend_from_slice(&map.image);
    let temporary = path.with_extension("tmp");
    fs::write(&temporary, &bytes)?;
    fs::rename(&temporary, path)
}

pub fn read(path: &Path) -> Option<Map> {
    let bytes = fs::read(path).ok()?;
    if bytes.len() != HEADER + MAP_BYTES || &bytes[..4] != MAGIC {
        return None;
    }
    Some(Map {
        route_id: u64::from_le_bytes(bytes[4..12].try_into().ok()?),
        rendered_ns: u64::from_le_bytes(bytes[12..20].try_into().ok()?),
        valid: bytes[20] == 1,
        image: bytes[HEADER..].to_vec(),
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn round_trips() {
        let path = std::env::temp_dir().join(format!("nav-map-{}", std::process::id()));
        let map = super::Map { route_id: 7, rendered_ns: 99, valid: true, image: vec![3; super::MAP_BYTES] };
        super::write(&path, &map).unwrap();
        assert_eq!(super::read(&path).unwrap(), map);
        let _ = std::fs::remove_file(path);
    }
}
