//! The selectable internal driving models (`InternalDrivingModel`).
//!
//! CD210 ships in the runtime bundle (`driving-models`); every other model is
//! a separate native catalog next to it. A model that fails to load keeps the
//! drive on CD210, and InternalDrivingModelStatus reports both ids.
use openpilot_model_runtime::catalog::Bundle;

#[derive(Debug, PartialEq, Eq)]
pub struct Model {
    /// Stable id written to InternalDrivingModelStatus and used by the web.
    pub id: &'static str,
    pub name: &'static str,
    /// Catalog directory next to `driving-models`.
    pub directory: &'static str,
}

pub const CD210: Model = Model {
    id: "cd210",
    name: "CD210",
    directory: "driving-models",
};

/// Param value -> model. Values not listed select CD210.
pub const MODELS: [Model; 3] = [
    CD210,
    // carrot-wip 1a307a02e, ONNX 350a47ea…: recurrent state inside the graph.
    Model {
        id: "cheddar",
        name: "Cheddar",
        directory: "driving-models-cheddar",
    },
    // commaai 62bf6fb07 (sunnypilot "Pop Model v2"): CD210's vision with
    // policy 05a58a51, combined with CD210's recipe (combine_split_model.py).
    Model {
        id: "popv2",
        name: "POPv2",
        directory: "driving-models-popv2",
    },
];

/// POPv2's policy checkpoint, as it appears in the combined model metadata.
const POPV2_POLICY: &str = "05a58a51-e0e3-4e9b-8e27-e644685f2c50";

#[must_use]
pub fn selected(value: i32) -> &'static Model {
    usize::try_from(value)
        .ok()
        .and_then(|index| MODELS.get(index))
        .unwrap_or(&MODELS[0])
}

/// The model a loaded bundle actually runs, from its contract and checkpoint.
#[must_use]
pub fn identify(bundle: &Bundle) -> &'static Model {
    if bundle
        .descriptor
        .inputs
        .iter()
        .any(|input| input.name == "state_img_q")
    {
        &MODELS[1]
    } else if bundle
        .descriptor
        .metadata
        .model_checkpoint
        .contains(POPV2_POLICY)
    {
        &MODELS[2]
    } else {
        &MODELS[0]
    }
}

#[cfg(test)]
mod tests {
    use super::{selected, MODELS};

    #[test]
    fn param_values_map_to_models_and_unknown_values_to_cd210() {
        assert_eq!(selected(0).id, "cd210");
        assert_eq!(selected(1).id, "cheddar");
        assert_eq!(selected(2).id, "popv2");
        assert_eq!(selected(7).id, "cd210");
        assert_eq!(selected(-1).id, "cd210");
        assert_eq!(MODELS[0].directory, "driving-models");
    }
}
