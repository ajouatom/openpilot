//! Every packaged eGPU model's output layout must parse with the shared
//! driving parser, so a model whose layout differs fails here instead of on
//! the first frame in a car (Cheddar's four-value action did exactly that).
use openpilot_modeld::{parse::RawOutputs, prediction::DrivingPrediction};
use std::{collections::BTreeMap, fs, path::Path};

#[test]
fn every_packaged_egpu_model_layout_parses() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../device-inputs/usbgpu-models");
    let mut checked = 0;
    for entry in fs::read_dir(&root).expect("packaged eGPU models") {
        let meta = entry.unwrap().path().join("model.hcq-meta.json");
        if !meta.is_file() {
            continue;
        }
        let value: serde_json::Value = serde_json::from_slice(&fs::read(&meta).unwrap()).unwrap();
        let count = value["output_count"].as_u64().unwrap() as usize;
        let slices: BTreeMap<String, [usize; 2]> = value["output_slices"]
            .as_object()
            .unwrap()
            .iter()
            .map(|(name, range)| {
                let start = range[0].as_u64().unwrap() as usize;
                let stop = range[1].as_u64().unwrap() as usize;
                (name.clone(), [start, stop])
            })
            .collect();
        let values = vec![0.0_f32; count];
        let raw = RawOutputs::new(&values, &slices)
            .unwrap_or_else(|error| panic!("{}: {error}", meta.display()));
        let prediction = DrivingPrediction::parse(&raw)
            .unwrap_or_else(|error| panic!("{}: {error}", meta.display()));
        if slices.contains_key("action") {
            assert!(prediction.direct_action.is_some(), "{}", meta.display());
        }
        checked += 1;
    }
    assert!(
        checked >= 4,
        "expected the four selectable eGPU models, found {checked}"
    );
}
