//! PMTiles v3 over any byte-range source (carrot `pmtiles.py`).
use crate::Error;
use flate2::read::GzDecoder;
use std::{collections::HashMap, io::Read};

pub fn read_varint(data: &[u8], mut offset: usize) -> Result<(u64, usize), Error> {
    let mut value = 0u64;
    for shift in (0..70).step_by(7) {
        let byte = *data.get(offset).ok_or(Error::Format("truncated varint"))?;
        offset += 1;
        if shift == 63 && byte > 1 {
            return Err(Error::Format("varint exceeds 64 bits"));
        }
        value |= u64::from(byte & 127) << shift;
        if byte < 128 {
            return Ok((value, offset));
        }
    }
    Err(Error::Format("varint exceeds 64 bits"))
}

/// Tile id along each zoom level's Hilbert curve.
pub fn tile_id(z: u8, x: u64, y: u64) -> Result<u64, Error> {
    if z > 26 || x >= 1 << z || y >= 1 << z {
        return Err(Error::Format("invalid tile coordinates"));
    }
    // The reference algorithm rotates with signed values that can go negative.
    let (mut x, mut y) = (x as i64, y as i64);
    let mut distance = 0i64;
    let mut size = (1i64 << z) / 2;
    while size > 0 {
        let rx = i64::from(x & size != 0);
        let ry = i64::from(y & size != 0);
        distance += size * size * ((3 * rx) ^ ry);
        if ry == 0 {
            if rx == 1 {
                x = size - 1 - x;
                y = size - 1 - y;
            }
            std::mem::swap(&mut x, &mut y);
        }
        size /= 2;
    }
    Ok(((1u64 << (2 * u32::from(z))) - 1) / 3 + distance as u64)
}

/// (ids, (run, length, offset)) of a decompressed directory.
pub type Directory = (Vec<u64>, Vec<(u64, u64, u64)>);

pub fn decode_directory(data: &[u8]) -> Result<Directory, Error> {
    let (count, mut offset) = read_varint(data, 0)?;
    if count as usize > data.len() {
        return Err(Error::Format("invalid directory entry count"));
    }
    let count = count as usize;
    let mut columns = [vec![0; count], vec![0; count], vec![0; count], vec![0; count]];
    for column in &mut columns {
        for value in column.iter_mut() {
            let (v, next) = read_varint(data, offset)?;
            *value = v;
            offset = next;
        }
    }
    let [mut ids, runs, lengths, mut offsets] = columns;
    for i in 0..count {
        if i > 0 {
            ids[i] = ids[i].checked_add(ids[i - 1]).ok_or(Error::Format("directory id overflow"))?;
        }
        if offsets[i] == 0 {
            if i == 0 {
                return Err(Error::Format("first directory offset cannot be implicit"));
            }
            offsets[i] = offsets[i - 1] + lengths[i - 1];
        } else {
            offsets[i] -= 1;
        }
        if lengths[i] == 0 || (i > 0 && ids[i] <= ids[i - 1]) {
            return Err(Error::Format("invalid directory entry"));
        }
    }
    if offset != data.len() {
        return Err(Error::Format("trailing directory data"));
    }
    let entries = (0..count).map(|i| (runs[i], lengths[i], offsets[i])).collect();
    Ok((ids, entries))
}

pub fn gunzip(data: &[u8], limit: usize) -> Result<Vec<u8>, Error> {
    let mut out = Vec::new();
    GzDecoder::new(data).take(limit as u64 + 1).read_to_end(&mut out)?;
    if out.len() > limit {
        return Err(Error::Format("decompressed data exceeds its limit"));
    }
    Ok(out)
}

/// A byte-range source: a local file or a remote archive.
pub trait Source {
    fn size(&self) -> u64;
    fn read(&mut self, offset: u64, length: u64) -> Result<Vec<u8>, Error>;
}

pub struct PmTiles<S: Source> {
    pub source: S,
    pub min_zoom: u8,
    pub max_zoom: u8,
    root: (u64, u64),
    leaf: (u64, u64),
    data: (u64, u64),
    directories: HashMap<(u64, u64), Directory>,
}

impl<S: Source> PmTiles<S> {
    pub fn open(mut source: S) -> Result<Self, Error> {
        let size = source.size();
        let header = source.read(0, 127)?;
        if header.len() != 127 || &header[..8] != b"PMTiles\x03" {
            return Err(Error::Format("expected a PMTiles v3 archive"));
        }
        let q = |i: usize| u64::from_le_bytes(header[8 + i * 8..16 + i * 8].try_into().expect("8 bytes"));
        let (root, leaf, data) = ((q(0), q(1)), (q(4), q(5)), (q(6), q(7)));
        if header[97..99] != [2, 2] || header[99] != 1 {
            return Err(Error::Format("expected gzip-compressed MVT vector tiles"));
        }
        let (min_zoom, max_zoom) = (header[100], header[101]);
        if min_zoom > max_zoom || max_zoom > 26 {
            return Err(Error::Format("unsupported PMTiles zoom range"));
        }
        for (offset, length) in [root, leaf, data] {
            if offset.checked_add(length).is_none_or(|end| end > size) {
                return Err(Error::Format("PMTiles section outside archive"));
            }
        }
        let mut archive = Self { source, min_zoom, max_zoom, root, leaf, data, directories: HashMap::new() };
        archive.directory(root.0, root.1)?;
        Ok(archive)
    }

    fn directory(&mut self, offset: u64, length: u64) -> Result<&Directory, Error> {
        if !self.directories.contains_key(&(offset, length)) {
            if self.directories.len() >= 32 {
                self.directories.clear();
            }
            let raw = self.source.read(offset, length)?;
            let directory = decode_directory(&gunzip(&raw, 64 << 20)?)?;
            self.directories.insert((offset, length), directory);
        }
        Ok(&self.directories[&(offset, length)])
    }

    /// The tile's byte range in the archive, or None when there is none.
    pub fn locate(&mut self, z: u8, x: u64, y: u64) -> Result<Option<(u64, u64)>, Error> {
        if !(self.min_zoom..=self.max_zoom).contains(&z) {
            return Ok(None);
        }
        let target = tile_id(z, x, y)?;
        let (mut offset, mut length) = self.root;
        for _ in 0..4 {
            let (data, leaf) = (self.data, self.leaf);
            let (ids, entries) = self.directory(offset, length)?;
            let index = ids.partition_point(|&id| id <= target);
            if index == 0 {
                return Ok(None);
            }
            let (run, l, o) = entries[index - 1];
            if run > 0 {
                if target >= ids[index - 1] + run {
                    return Ok(None);
                }
                if o + l > data.1 {
                    return Err(Error::Format("tile outside PMTiles data section"));
                }
                return Ok(Some((data.0 + o, l)));
            }
            if o + l > leaf.1 {
                return Err(Error::Format("directory outside PMTiles leaf section"));
            }
            offset = leaf.0 + o;
            length = l;
        }
        Err(Error::Format("PMTiles directory nesting exceeds four levels"))
    }

    /// The decompressed MVT tile, or None.
    pub fn get(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error> {
        let Some((offset, length)) = self.locate(z, x, y)? else {
            return Ok(None);
        };
        let raw = self.source.read(offset, length)?;
        Ok(Some(gunzip(&raw, 16 << 20)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hilbert_ids_match_the_pmtiles_spec() {
        assert_eq!(tile_id(0, 0, 0).unwrap(), 0);
        assert_eq!(tile_id(1, 0, 0).unwrap(), 1);
        assert_eq!(tile_id(1, 0, 1).unwrap(), 2);
        assert_eq!(tile_id(1, 1, 1).unwrap(), 3);
        assert_eq!(tile_id(1, 1, 0).unwrap(), 4);
        assert_eq!(tile_id(2, 0, 0).unwrap(), 5);
        assert!(tile_id(1, 2, 0).is_err());
    }

    #[test]
    fn directory_decoding_resolves_implicit_offsets() {
        // 2 entries: ids [5, +1], runs [1, 1], lengths [10, 20], offsets [1 (=0), 0 (implicit)]
        let data = [2, 5, 1, 1, 1, 10, 20, 1, 0];
        let (ids, entries) = decode_directory(&data).unwrap();
        assert_eq!(ids, vec![5, 6]);
        assert_eq!(entries, vec![(1, 10, 0), (1, 20, 10)]);
        assert!(decode_directory(&[1, 5, 1, 10, 0]).is_err());
    }
}
