//! Road polylines from Protomaps or Mapbox Streets v8 MVT tiles without a
//! protobuf runtime (carrot `vector_tile.py`).
use crate::{pmtiles::read_varint, Error};

#[derive(Clone, Debug, PartialEq)]
pub struct Road {
    /// Tile coordinates, x right / y down.
    pub points: Vec<[f64; 2]>,
    pub kind: String,
    pub extent: u32,
    pub detail: String,
    pub tunnel: bool,
    pub bridge: bool,
}

#[derive(Clone, Debug, PartialEq)]
enum Value {
    Text(String),
    Bool(bool),
    Other,
}

struct Fields<'a> {
    data: &'a [u8],
    offset: usize,
}

enum Field<'a> {
    Varint(u64),
    Bytes(&'a [u8]),
}

impl<'a> Iterator for Fields<'a> {
    type Item = Result<(u64, Field<'a>), Error>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.data.len() {
            return None;
        }
        Some((|| {
            let (tag, offset) = read_varint(self.data, self.offset)?;
            self.offset = offset;
            let (number, wire) = (tag >> 3, tag & 7);
            if number == 0 {
                return Err(Error::Format("invalid protobuf field"));
            }
            let field = match wire {
                0 => {
                    let (value, offset) = read_varint(self.data, self.offset)?;
                    self.offset = offset;
                    Field::Varint(value)
                }
                1 | 2 | 5 => {
                    let length = if wire == 2 {
                        let (length, offset) = read_varint(self.data, self.offset)?;
                        self.offset = offset;
                        length as usize
                    } else if wire == 1 {
                        8
                    } else {
                        4
                    };
                    let end = self.offset.checked_add(length).filter(|&e| e <= self.data.len())
                        .ok_or(Error::Format("truncated protobuf field"))?;
                    let bytes = &self.data[self.offset..end];
                    self.offset = end;
                    Field::Bytes(bytes)
                }
                _ => return Err(Error::Format("unsupported protobuf wire type")),
            };
            Ok((number, field))
        })())
    }
}

fn fields(data: &[u8]) -> Fields<'_> {
    Fields { data, offset: 0 }
}

fn integers(data: &[u8], out: &mut Vec<u64>) -> Result<(), Error> {
    let mut offset = 0;
    while offset < data.len() {
        let (value, next) = read_varint(data, offset)?;
        out.push(value);
        offset = next;
    }
    Ok(())
}

fn unzigzag(value: u64) -> i64 {
    ((value >> 1) as i64) ^ -((value & 1) as i64)
}

fn decode_value(data: &[u8]) -> Result<Value, Error> {
    for field in fields(data) {
        match field? {
            (1, Field::Bytes(text)) => {
                return Ok(Value::Text(String::from_utf8_lossy(text).into_owned()))
            }
            (7, Field::Varint(v)) => return Ok(Value::Bool(v != 0)),
            _ => {}
        }
    }
    Ok(Value::Other)
}

fn decode_lines(commands: &[u64]) -> Result<Vec<Vec<[f64; 2]>>, Error> {
    let mut lines = Vec::new();
    let mut line: Vec<[f64; 2]> = Vec::new();
    let (mut x, mut y, mut offset) = (0i64, 0i64, 0usize);
    while offset < commands.len() {
        let command = commands[offset];
        offset += 1;
        let (kind, count) = (command & 7, (command >> 3) as usize);
        if !(kind == 1 || kind == 2) || count == 0 || offset + 2 * count > commands.len() {
            return Err(Error::Format("invalid MVT line geometry"));
        }
        if kind == 1 && count != 1 {
            return Err(Error::Format("LineString MoveTo must contain one point"));
        }
        if kind == 2 && line.is_empty() {
            return Err(Error::Format("LineTo before MoveTo"));
        }
        for _ in 0..count {
            x += unzigzag(commands[offset]);
            y += unzigzag(commands[offset + 1]);
            offset += 2;
            if kind == 1 {
                if line.len() >= 2 {
                    lines.push(std::mem::take(&mut line));
                }
                line.clear();
            }
            line.push([x as f64, y as f64]);
        }
    }
    if line.len() >= 2 {
        lines.push(line);
    }
    Ok(lines)
}

pub fn decode_roads(data: &[u8]) -> Result<Vec<Road>, Error> {
    let mut roads = Vec::new();
    for field in fields(data) {
        let (3, Field::Bytes(layer)) = field? else { continue };
        let mut name = String::new();
        let (mut version, mut extent) = (1u64, 4096u64);
        let (mut keys, mut values, mut features) = (Vec::new(), Vec::new(), Vec::new());
        for f in fields(layer) {
            match f? {
                (1, Field::Bytes(v)) => name = String::from_utf8_lossy(v).into_owned(),
                (15, Field::Varint(v)) => version = v,
                (5, Field::Varint(v)) => extent = v,
                (3, Field::Bytes(v)) => keys.push(String::from_utf8_lossy(v).into_owned()),
                (4, Field::Bytes(v)) => values.push(decode_value(v)?),
                (2, Field::Bytes(v)) => features.push(v),
                _ => {}
            }
        }
        if name != "roads" && name != "road" {
            continue;
        }
        if !(1..=2).contains(&version) || extent == 0 || extent > u64::from(u32::MAX) {
            return Err(Error::Format("unsupported MVT roads layer"));
        }
        for feature in features {
            let (mut tags, mut geometry, mut geometry_type) = (Vec::new(), Vec::new(), 0);
            for f in fields(feature) {
                match f? {
                    (2, Field::Bytes(v)) => integers(v, &mut tags)?,
                    (2, Field::Varint(v)) => tags.push(v),
                    (4, Field::Bytes(v)) => integers(v, &mut geometry)?,
                    (4, Field::Varint(v)) => geometry.push(v),
                    (3, Field::Varint(v)) => geometry_type = v,
                    _ => {}
                }
            }
            if geometry_type != 2 {
                continue;
            }
            if tags.len() % 2 != 0 {
                return Err(Error::Format("unpaired MVT tags"));
            }
            let text = |key: &str| -> Option<&Value> {
                tags.chunks(2).find_map(|pair| {
                    let (k, v) = (pair[0] as usize, pair[1] as usize);
                    (keys.get(k).map(String::as_str) == Some(key)).then(|| values.get(v)).flatten()
                })
            };
            for pair in tags.chunks(2) {
                if pair[0] as usize >= keys.len() || pair[1] as usize >= values.len() {
                    return Err(Error::Format("MVT tag index out of bounds"));
                }
            }
            let string = |v: Option<&Value>| match v {
                Some(Value::Text(t)) => t.clone(),
                _ => String::new(),
            };
            let (kind, detail, tunnel, bridge) = if name == "road" {
                // Mapbox Streets v8: class/structure instead of Protomaps kind.
                let detail = string(text("class"));
                let kind = match detail.as_str() {
                    "motorway" | "trunk" | "primary" | "motorway_link" | "trunk_link" | "primary_link" => "highway",
                    "secondary" | "tertiary" | "secondary_link" | "tertiary_link" => "major_road",
                    "street" | "street_limited" | "service" | "track" => "minor_road",
                    // rail, pedestrian, path, ferry, construction: not driveable
                    _ => continue,
                };
                let structure = string(text("structure"));
                (kind.to_owned(), detail, structure == "tunnel", structure == "bridge")
            } else {
                let kind = string(text("kind"));
                if !matches!(kind.as_str(), "highway" | "major_road" | "minor_road" | "other") {
                    continue;
                }
                let detail = string(text("kind_detail"));
                let tunnel = text("is_tunnel") == Some(&Value::Bool(true));
                let bridge = text("is_bridge") == Some(&Value::Bool(true));
                (kind, detail, tunnel, bridge)
            };
            for points in decode_lines(&geometry)? {
                roads.push(Road {
                    points,
                    kind: kind.clone(),
                    extent: extent as u32,
                    detail: detail.clone(),
                    tunnel,
                    bridge,
                });
            }
        }
    }
    Ok(roads)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn varint(mut v: u64, out: &mut Vec<u8>) {
        loop {
            let byte = (v & 127) as u8;
            v >>= 7;
            if v == 0 {
                out.push(byte);
                return;
            }
            out.push(byte | 128);
        }
    }
    fn bytes(tag: u64, data: &[u8], out: &mut Vec<u8>) {
        varint(tag << 3 | 2, out);
        varint(data.len() as u64, out);
        out.extend_from_slice(data);
    }

    #[test]
    fn decodes_a_protomaps_bridge_road() {
        let mut kind = Vec::new();
        bytes(1, b"highway", &mut kind);
        let mut yes = Vec::new();
        varint(7 << 3, &mut yes);
        yes.push(1);
        let mut tags = Vec::new();
        for v in [0, 0, 1, 1] {
            varint(v, &mut tags);
        }
        // MoveTo(1) (2,3), LineTo(1) (+4,+0)
        let mut geometry = Vec::new();
        for v in [9, 4, 6, 10, 8, 0] {
            varint(v, &mut geometry);
        }
        let mut feature = Vec::new();
        bytes(2, &tags, &mut feature);
        varint(3 << 3, &mut feature);
        feature.push(2);
        bytes(4, &geometry, &mut feature);
        let mut layer = Vec::new();
        bytes(1, b"roads", &mut layer);
        bytes(3, b"kind", &mut layer);
        bytes(3, b"is_bridge", &mut layer);
        bytes(4, &kind, &mut layer);
        bytes(4, &yes, &mut layer);
        bytes(2, &feature, &mut layer);
        let mut tile = Vec::new();
        bytes(3, &layer, &mut tile);
        let roads = decode_roads(&tile).unwrap();
        assert_eq!(roads.len(), 1);
        assert_eq!(roads[0].points, vec![[2., 3.], [6., 3.]]);
        assert_eq!(roads[0].kind, "highway");
        assert!(roads[0].bridge && !roads[0].tunnel);
        assert_eq!(roads[0].extent, 4096);
    }
}
