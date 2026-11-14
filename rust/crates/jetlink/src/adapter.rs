use crate::{contract, Deadline, Error};
use openpilot_modeld::{parse::RawOutputs, prediction::DrivingPrediction};
use std::collections::BTreeMap;

pub struct Adapter {
    previous_desire: [f32; 8],
    reset_next: bool,
    slices: BTreeMap<String, [usize; 2]>,
    output_count: usize,
    plan: [usize; 2],
    hidden: Option<[usize; 2]>,
    previous_features: Vec<f32>,
    /// A navigation graph (N16): `navigation` follows the 12 scalars.
    navigation: bool,
}
impl Adapter {
    pub fn new() -> Result<Self, Error> {
        Self::for_spec(&crate::model_spec::ModelSpec::parse(
            contract::CONTRACT_JSON.as_bytes(),
        )?)
    }
    pub fn for_spec(spec: &crate::model_spec::ModelSpec) -> Result<Self, Error> {
        spec.validate()?;
        let mut slices: BTreeMap<String, [usize; 2]> = spec
            .output_slices
            .keys()
            .map(|name| Ok((name.clone(), spec.range(name)?)))
            .collect::<Result<_, Error>>()?;
        // Original get_action consumes action[0,0:2]; retain all raw values on the wire.
        if let Some(section) = slices.get_mut("action") {
            section[1] = section[0] + 2;
        }
        Ok(Self {
            hidden: if spec.stateful() {
                None
            } else {
                Some(spec.range("hidden_state")?)
            },
            previous_features: if spec.stateful() {
                Vec::new()
            } else {
                vec![0.; spec.range("hidden_state")?[1] - spec.range("hidden_state")?[0]]
            },
            previous_desire: [0.0; 8],
            navigation: spec.navigation(),
            reset_next: true,
            plan: slices["plan"],
            slices,
            output_count: spec.output_count()?,
        })
    }
    pub fn reset(&mut self) {
        self.previous_desire.fill(0.0);
        self.previous_features.fill(0.0);
        self.reset_next = true;
    }
    pub fn reset_next(&self) -> bool {
        self.reset_next
    }
    pub fn prepare(
        &mut self,
        warped: &[u8],
        mut desire: [f32; 8],
        traffic: [f32; 2],
        action: [f32; 2],
        prepare_only: bool,
    ) -> Result<Option<[f32; 12]>, Error> {
        if prepare_only {
            self.reset();
            return Ok(None);
        }
        if warped.len() != contract::WARPED_BYTES {
            return Err(Error::Contract("warped camera shape"));
        }
        desire[0] = 0.0;
        let mut packed = [0.0; 12];
        for i in 0..8 {
            packed[i] = if desire[i] - self.previous_desire[i] > 0.99 {
                desire[i]
            } else {
                0.0
            };
        }
        self.previous_desire = desire;
        packed[8..10].copy_from_slice(&traffic);
        packed[10..12].copy_from_slice(&action);
        contract::check_input(warped, &packed)?;
        Ok(Some(packed))
    }
    pub fn prepare_session(
        &mut self,
        warped: &[u8],
        desire: [f32; 8],
        traffic: [f32; 2],
        action: [f32; 2],
        navigation: &[f32; crate::model_spec::NAVIGATION_FLOATS],
        packed_count: usize,
    ) -> Result<Vec<f32>, Error> {
        let extra = if self.navigation {
            crate::model_spec::NAVIGATION_FLOATS
        } else {
            self.previous_features.len()
        };
        if packed_count != 12 && packed_count != 12 + extra {
            return Err(Error::Contract("session recurrent packed size"));
        }
        if self.navigation && !navigation.iter().all(|v| v.is_finite()) {
            return Err(Error::Contract("navigation input finite"));
        }
        let controls = self
            .prepare(warped, desire, traffic, action, false)?
            .ok_or(Error::Contract("prepared controls missing"))?;
        let mut packed = Vec::with_capacity(packed_count);
        packed.extend_from_slice(&controls);
        if packed_count > 12 {
            if self.navigation {
                packed.extend_from_slice(navigation);
            } else {
                packed.extend_from_slice(&self.previous_features);
            }
        }
        Ok(packed)
    }
    pub fn parse(
        &mut self,
        output: &[f32],
        deadline: Deadline,
    ) -> Result<DrivingPrediction, Error> {
        deadline.remaining()?;
        if output.len() != self.output_count || !output.iter().all(|v| v.is_finite()) {
            return Err(Error::Contract("selected model output shape/finite"));
        }
        let mut plan = self.plan;
        if plan[1] - plan[0] == 4955 {
            let winner = crate::plan_hypothesis::winner(std::array::from_fn(|i| {
                output[plan[0] + i * 991 + 990]
            }));
            plan[0] += winner * 991;
            plan[1] = plan[0] + 990;
        }
        *self.slices.get_mut("plan").expect("validated plan") = plan;
        let parsed = DrivingPrediction::parse(&RawOutputs::new(output, &self.slices)?)?;
        if let Some([start, stop]) = self.hidden {
            self.previous_features.copy_from_slice(&output[start..stop]);
        }
        self.reset_next = false;
        deadline.remaining()?;
        Ok(parsed)
    }
}
