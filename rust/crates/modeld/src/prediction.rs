use crate::parse::{sigmoid, softmax, Normal, ParseError, RawOutputs};

#[derive(Debug)]
pub struct DrivingPrediction {
    pub plan: [[f32; 15]; 33],
    pub plan_std: [[f32; 15]; 33],
    pub pose: Normal<6>,
    pub wide_euler: Normal<3>,
    pub road_transform: Normal<6>,
    pub lanes: Normal<264>,
    pub edges: Normal<132>,
    pub leads: Normal<72>,
    pub lane_prob: [f32; 8],
    pub lead_prob: [f32; 3],
    pub meta: [f32; 55],
    pub desire_state: [f32; 8],
    pub desire_prediction: [f32; 32],
    pub direct_action: Option<[f32; 2]>,
}

/// Lateral acceleration and acceleration, the first two action values.
/// Cheddar returns four; carrot's modeld reads `action[0,0]` and `action[0,1]`.
fn direct_action(values: &[f32]) -> Result<[f32; 2], ParseError> {
    match values {
        [lateral, acceleration, ..] => Ok([*lateral, *acceleration]),
        _ => Err(ParseError::Width {
            name: "action".to_owned(),
            actual: values.len(),
            expected: 2,
        }),
    }
}

impl DrivingPrediction {
    pub fn parse(raw: &RawOutputs<'_>) -> Result<Self, ParseError> {
        let plan = raw.normal::<495>("plan")?;
        let mut desire_state = raw.array("desire_state")?;
        softmax(&mut desire_state);
        let mut desire_prediction = raw.array::<32>("desire_pred")?;
        for row in desire_prediction.chunks_exact_mut(8) {
            softmax(row);
        }
        Ok(Self {
            plan: std::array::from_fn(|i| std::array::from_fn(|j| plan.mean[i * 15 + j])),
            plan_std: std::array::from_fn(|i| std::array::from_fn(|j| plan.std[i * 15 + j])),
            pose: raw.normal("pose")?,
            wide_euler: raw.normal("wide_from_device_euler")?,
            road_transform: raw.normal("road_transform")?,
            lanes: raw.normal("lane_lines")?,
            edges: raw.normal("road_edges")?,
            leads: raw.leads()?,
            lane_prob: raw.array::<8>("lane_lines_prob")?.map(sigmoid),
            lead_prob: raw.array::<3>("lead_prob")?.map(sigmoid),
            meta: raw.array::<55>("meta")?.map(sigmoid),
            desire_state,
            desire_prediction,
            direct_action: if raw.has("action") {
                Some(direct_action(raw.get("action")?)?)
            } else {
                None
            },
        })
    }
}

#[derive(Debug)]
pub struct DriverData {
    pub face: Normal<6>,
    pub probabilities: [f32; 8],
}

impl DriverData {
    fn parse(raw: &RawOutputs<'_>, suffix: &str) -> Result<Self, ParseError> {
        let face = raw.normal(&format!("face_descs_{suffix}"))?;
        let mut probabilities = [0.0; 8];
        for (value, name) in probabilities.iter_mut().zip([
            "face_prob",
            "left_eye_prob",
            "right_eye_prob",
            "left_blink_prob",
            "right_blink_prob",
            "sunglasses_prob",
            "using_phone_prob",
            "sleep_prob",
        ]) {
            *value = sigmoid(raw.array::<1>(&format!("{name}_{suffix}"))?[0]);
        }
        Ok(Self {
            face,
            probabilities,
        })
    }
}

#[derive(Debug)]
pub struct DriverPrediction {
    pub left: DriverData,
    pub right: DriverData,
    pub wheel_on_right: f32,
}

impl DriverPrediction {
    pub fn parse(raw: &RawOutputs<'_>) -> Result<Self, ParseError> {
        Ok(Self {
            left: DriverData::parse(raw, "lhd")?,
            right: DriverData::parse(raw, "rhd")?,
            wheel_on_right: sigmoid(raw.array::<1>("wheel_on_right")?[0]),
        })
    }
}

#[cfg(test)]
mod cheddar_tests {
    use super::*;
    use std::collections::BTreeMap;

    /// Cheddar's output_slices metadata (outputs [1, 2580]).
    fn cheddar_slices() -> BTreeMap<String, [usize; 2]> {
        [
            ("lane_lines", [0, 528]),
            ("lane_lines_prob", [528, 536]),
            ("road_edges", [536, 800]),
            ("meta", [800, 855]),
            ("desire_pred", [855, 887]),
            ("pose", [887, 899]),
            ("wide_from_device_euler", [899, 905]),
            ("road_transform", [905, 917]),
            ("plan", [917, 1907]),
            ("lead", [1907, 2051]),
            ("lead_prob", [2051, 2054]),
            ("desire_state", [2054, 2062]),
            ("action", [2062, 2066]),
            ("hidden_state", [2066, 2578]),
            ("pad", [2578, 2580]),
        ]
        .into_iter()
        .map(|(name, range)| (name.to_owned(), range))
        .collect()
    }

    #[test]
    fn cheddar_layout_parses_and_uses_the_first_two_actions() {
        let slices = cheddar_slices();
        let mut values = vec![0.0_f32; 2580];
        values[2062..2066].copy_from_slice(&[0.25, -0.5, 9.0, 9.0]);
        values[917 + 15] = 3.0; // plan row 1, x position
        let raw = RawOutputs::new(&values, &slices).unwrap();
        let prediction = DrivingPrediction::parse(&raw).unwrap();
        assert_eq!(prediction.direct_action, Some([0.25, -0.5]));
        assert_eq!(prediction.plan[1][0], 3.0);
    }

    #[test]
    fn a_single_action_value_is_rejected() {
        assert!(direct_action(&[1.0]).is_err());
        assert_eq!(direct_action(&[1.0, 2.0]).unwrap(), [1.0, 2.0]);
    }
}
