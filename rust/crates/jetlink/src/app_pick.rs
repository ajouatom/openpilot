//! The driving model picked on the comma (carrot web's model library) for an
//! App host. The App can switch models whenever its owner asks, so a pick made
//! here is not a next-boot setting: it rides in the hello (`pick`, protocol 3),
//! the App keeps the newer of its own pick and ours (PickStore), and when ours
//! wins the comma asks for that engine and uploads the ONNX if the App has not
//! got it. carrot-server writes the file; this daemon only reads it.
use crate::{model_spec::ModelSpec, Error};
use serde::{Deserialize, Serialize};
use std::{
    io::{Read, Write},
    path::Path,
};

pub const FILE: &str = "app-pick.json";
/// `at` of a pick whose download, upload or build failed: it is not named
/// again (no 800 MB download per reconnect) until a new pick is made.
const FAILED: &str = "app-pick-failed";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AppPick {
    /// Catalog key (`n16`), shown in the App as the pick's ref.
    pub model: String,
    pub name: String,
    /// The ONNX download for an App that has not got the model.
    pub url: String,
    /// Unix seconds by the comma's clock.
    pub at: f64,
    pub spec: ModelSpec,
}

impl AppPick {
    /// The hello's `pick`: what upstream PickStore.meet reads.
    pub fn hello(&self, now: f64) -> serde_json::Value {
        serde_json::json!({
            "ref": self.model, "sha256": self.spec.sha256, "nbytes": self.spec.nbytes,
            "name": self.name, "age_s": (now - self.at).max(0.), "at": self.at,
        })
    }
    /// The App's answer names this pick: ours is the newer one.
    pub fn won(&self, reply: &serde_json::Value) -> bool {
        reply["pick"]["sha256"].as_str() == Some(self.spec.sha256.as_str())
    }
}

/// None when there is no pick or it is unreadable; a bad file must never stop
/// the link, the App's own model is used then.
pub fn read(cache: &Path) -> Option<AppPick> {
    let mut bytes = Vec::new();
    std::fs::File::open(cache.join(FILE))
        .ok()?
        .take(1 << 20)
        .read_to_end(&mut bytes)
        .ok()?;
    let pick: AppPick = serde_json::from_slice(&bytes).ok()?;
    (pick.spec.validate().is_ok()
        && pick.at.is_finite()
        && pick.at > 0.
        && pick.url.starts_with("https://")
        && pick.model.len() <= 64
        && pick
            .model
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_'))
    .then_some(pick)
}

/// The pick to name in a hello: none after it failed here.
pub fn current(cache: &Path) -> Option<AppPick> {
    let pick = read(cache)?;
    let failed = std::fs::read_to_string(cache.join(FAILED)).ok();
    (failed.as_deref().map(str::trim) != Some(pick.at.to_string().as_str())).then_some(pick)
}

pub fn failed(cache: &Path, pick: &AppPick) {
    let _ = std::fs::write(cache.join(FAILED), pick.at.to_string());
}

pub fn write(cache: &Path, pick: &AppPick) -> Result<(), Error> {
    pick.spec.validate()?;
    std::fs::create_dir_all(cache)?;
    let mut staging = tempfile::NamedTempFile::new_in(cache)?;
    serde_json::to_writer(staging.as_file_mut(), pick)?;
    staging.flush()?;
    staging.as_file().sync_all()?;
    staging
        .persist(cache.join(FILE))
        .map_err(|error| error.error)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pick() -> AppPick {
        AppPick {
            model: "n16".into(),
            name: "N16 Nav".into(),
            url: "https://example.com/big_driving_supercombo.onnx".into(),
            at: 1_700_000_000.,
            spec: ModelSpec::parse(include_bytes!("../data/n16.json")).unwrap(),
        }
    }

    #[test]
    fn round_trip_and_hello() {
        let root = tempfile::tempdir().unwrap();
        assert!(read(root.path()).is_none());
        write(root.path(), &pick()).unwrap();
        let back = read(root.path()).unwrap();
        assert_eq!(back, pick());
        let hello = back.hello(1_700_000_010.);
        assert_eq!(hello["age_s"], 10.);
        assert_eq!(hello["ref"], "n16");
        assert!(back.won(&serde_json::json!({"pick":{"sha256":back.spec.sha256}})));
        assert!(!back.won(&serde_json::json!({"pick":null})));
        assert!(current(root.path()).is_some());
        failed(root.path(), &back);
        assert!(current(root.path()).is_none());
        let mut again = pick();
        again.at += 1.;
        write(root.path(), &again).unwrap();
        assert_eq!(current(root.path()), Some(again));
    }

    #[test]
    fn rejects_plain_http_and_bad_ref() {
        let root = tempfile::tempdir().unwrap();
        let mut bad = pick();
        bad.url = "http://example.com/x.onnx".into();
        write(root.path(), &bad).unwrap();
        assert!(read(root.path()).is_none());
        let mut bad = pick();
        bad.model = "../x".into();
        write(root.path(), &bad).unwrap();
        assert!(read(root.path()).is_none());
    }
}
