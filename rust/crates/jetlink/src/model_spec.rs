//! Source driving graph contracts; protocol version and model generation are independent.
use crate::{wire::Version, Error};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const LEGACY_JSON: &str = include_str!("../data/cinque_v2.json");
/// `navigation` floats a navigation graph takes after the 12 scalars.
pub const NAVIGATION_FLOATS: usize = 47;
/// `nav_map` bytes (uint8 256x256) sent after the packed floats when a frame
/// carries a new map (wire flag NAV_MAP).
pub const NAV_MAP_BYTES: usize = 256 * 256;
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
pub struct ModelSpec {
    pub sha256: String,
    pub nbytes: u64,
    #[serde(default = "default_frame_skip")]
    pub frame_skip: usize,
    pub checkpoint: Option<String>,
    pub input_shapes: BTreeMap<String, Vec<usize>>,
    pub output_shapes: BTreeMap<String, Vec<usize>>,
    pub output_slices: BTreeMap<String, Vec<Option<i64>>>,
}
fn default_frame_skip() -> usize {
    4
}
fn count(shape: &[usize]) -> Result<usize, Error> {
    if shape.is_empty() || shape.contains(&0) {
        return Err(Error::Contract("model shape"));
    }
    shape.iter().try_fold(1usize, |total, &n| {
        total
            .checked_mul(n)
            .filter(|&v| v <= 32 << 20)
            .ok_or(Error::Contract("model shape size"))
    })
}
impl ModelSpec {
    pub fn parse(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() > 65536 {
            return Err(Error::Contract("model spec size"));
        }
        let value: Self = serde_json::from_slice(bytes)?;
        value.validate()?;
        Ok(value)
    }
    pub fn stateful(&self) -> bool {
        self.input_shapes.contains_key("new_img")
    }
    /// A navigation (NOO) graph such as carrot's N16: a 256x256 route map
    /// and the 47-float maneuver input beside the camera.
    pub fn navigation(&self) -> bool {
        self.input_shapes.contains_key("nav_map") || self.input_shapes.contains_key("navigation")
    }
    pub fn output_count(&self) -> Result<usize, Error> {
        count(
            self.output_shapes
                .get("outputs")
                .ok_or(Error::Contract("model output missing"))?,
        )
    }
    pub fn range(&self, name: &str) -> Result<[usize; 2], Error> {
        let section = self
            .output_slices
            .get(name)
            .ok_or(Error::Contract("model slice missing"))?;
        if !(1..=3).contains(&section.len())
            || section.get(2).copied().flatten().is_some_and(|v| v != 1)
        {
            return Err(Error::Contract("model slice step"));
        }
        let total = self.output_count()? as i64;
        let normalize = |value: Option<i64>, default| -> Result<usize, Error> {
            let value = value.unwrap_or(default);
            if !(-total..=total).contains(&value) {
                return Err(Error::Contract("model slice bounds"));
            }
            Ok(if value < 0 { total + value } else { value } as usize)
        };
        let (start, stop) = if section.len() == 1 {
            (None, section[0])
        } else {
            (section[0], section[1])
        };
        let result = [normalize(start, 0)?, normalize(stop, total)?];
        if result[0] >= result[1] {
            return Err(Error::Contract("model slice empty"));
        }
        Ok(result)
    }
    pub fn packed_count(&self, version: Version) -> Result<usize, Error> {
        if self.stateful() && version == Version::V2 {
            return Err(Error::Contract("stateful graph requires protocol 3"));
        }
        if version == Version::V3 {
            return Ok(12 + if self.navigation() { NAVIGATION_FLOATS } else { 0 });
        }
        let shape = self
            .input_shapes
            .get("features_buffer")
            .ok_or(Error::Contract("features missing"))?;
        count(&shape[2..]).and_then(|v| v.checked_add(12).ok_or(Error::Contract("packed size")))
    }
    pub fn validate(&self) -> Result<(), Error> {
        if self.sha256.len() != 64
            || !self
                .sha256
                .bytes()
                .all(|v| v.is_ascii_digit() || (b'a'..=b'f').contains(&v))
            || !(1..=4 << 30).contains(&self.nbytes)
            || self.frame_skip != 4
        {
            return Err(Error::Contract("model identity/context"));
        }
        for shape in self
            .input_shapes
            .values()
            .chain(self.output_shapes.values())
        {
            count(shape)?;
        }
        let shape = |name: &str, expected: &[usize]| {
            self.input_shapes.get(name).is_some_and(|v| v == expected)
        };
        if !shape("traffic_convention", &[1, 2]) || !shape("action_t", &[1, 2]) {
            return Err(Error::Contract("model scalar inputs"));
        }
        if self.stateful() {
            if !shape("new_img", &[2, 6, 128, 256])
                || self
                    .input_shapes
                    .get("desire")
                    .map_or(Ok(0), |v| count(v))?
                    != 8
            {
                return Err(Error::Contract("stateful camera/desire"));
            }
            if self.navigation()
                && (!shape("nav_map", &[1, 1, 256, 256]) || !shape("navigation", &[1, 47]))
            {
                return Err(Error::Contract("navigation inputs"));
            }
            let states = self
                .input_shapes
                .keys()
                .filter(|k| {
                    ![
                        "new_img",
                        "desire",
                        "traffic_convention",
                        "action_t",
                        "nav_map",
                        "navigation",
                    ]
                    .contains(&k.as_str())
                })
                .collect::<Vec<_>>();
            if states.is_empty()
                || states.iter().any(|name| {
                    !name.starts_with("state_")
                        || self.output_shapes.get(&format!("next_{name}"))
                            != self.input_shapes.get(*name)
                })
                || self.output_shapes.len() != states.len() + 1
            {
                return Err(Error::Contract("graph state contract"));
            }
        } else {
            let names = self
                .input_shapes
                .keys()
                .map(String::as_str)
                .collect::<BTreeSet<_>>();
            if names
                != BTreeSet::from([
                    "img",
                    "big_img",
                    "desire_pulse",
                    "features_buffer",
                    "traffic_convention",
                    "action_t",
                ])
                || self.output_shapes.len() != 1
                || !shape("img", &[1, 12, 128, 256])
                || !shape("big_img", &[1, 12, 128, 256])
            {
                return Err(Error::Contract("queued model inputs"));
            }
            let desire = &self.input_shapes["desire_pulse"];
            let features = &self.input_shapes["features_buffer"];
            if desire.len() != 3
                || desire[0] != 1
                || desire[2] != 8
                || features.len() < 3
                || features[0] != 1
            {
                return Err(Error::Contract("recurrent inputs"));
            }
            let hidden = self.range("hidden_state")?;
            if hidden[1] - hidden[0] != count(&features[2..])? {
                return Err(Error::Contract("hidden state size"));
            }
        }
        let outputs = self
            .output_shapes
            .get("outputs")
            .ok_or(Error::Contract("outputs missing"))?;
        if outputs.len() != 2 || outputs[0] != 1 || self.output_count()? > 65536 {
            return Err(Error::Contract("driving output shape"));
        }
        let mut ranges = self
            .output_slices
            .keys()
            .map(|k| self.range(k))
            .collect::<Result<Vec<_>, _>>()?;
        ranges.sort();
        if ranges.windows(2).any(|v| v[0][1] > v[1][0]) {
            return Err(Error::Contract("overlapping model slices"));
        }
        for (name, widths) in [
            ("lane_lines", &[528][..]),
            ("lane_lines_prob", &[8]),
            ("road_edges", &[264]),
            ("meta", &[55]),
            ("desire_pred", &[32]),
            ("pose", &[12]),
            ("wide_from_device_euler", &[6]),
            ("road_transform", &[12]),
            ("plan", &[990, 4955]),
            ("lead", &[144, 102]),
            ("lead_prob", &[3]),
            ("desire_state", &[8]),
        ] {
            let range = self.range(name)?;
            if !widths.contains(&(range[1] - range[0])) {
                return Err(Error::Contract("driving output semantics"));
            }
        }
        if self.output_slices.contains_key("action") {
            let r = self.range("action")?;
            if ![2, 4].contains(&(r[1] - r[0])) {
                return Err(Error::Contract("action output"));
            }
        }
        let legacy: Self = serde_json::from_str(LEGACY_JSON)?;
        if self.sha256 == legacy.sha256 {
            let mut canonical = self.clone();
            for section in canonical.output_slices.values_mut() {
                if section.len() == 1 {
                    section.insert(0, None);
                }
                section.truncate(2);
            }
            if canonical != legacy {
                return Err(Error::Contract("legacy model identity mismatch"));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod navigation_tests {
    use super::*;

    #[test]
    fn carrot_n16_is_a_valid_navigation_graph_with_59_packed_floats() {
        let spec = ModelSpec::parse(include_bytes!("../data/n16.json")).unwrap();
        assert!(spec.stateful() && spec.navigation());
        assert_eq!(spec.packed_count(Version::V3).unwrap(), 12 + NAVIGATION_FLOATS);
        // Half a navigation contract, or the wrong map shape, is refused.
        let mut half = spec.clone();
        half.input_shapes.remove("nav_map");
        assert!(half.validate().is_err());
        let mut wrong = spec.clone();
        wrong.input_shapes.insert("nav_map".into(), vec![1, 1, 128, 128]);
        assert!(wrong.validate().is_err());
        let legacy = ModelSpec::parse(LEGACY_JSON.as_bytes()).unwrap();
        assert!(!legacy.navigation());
    }
}
