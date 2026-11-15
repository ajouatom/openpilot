//! Original web_settings.py and web_capabilities.py policies and persistence.
mod capabilities;
mod catalog;
mod catalog_cache;
mod coercion;
mod layout;
mod schema;

use crate::{json_fields::set, Error, Value};
pub use capabilities::{capability_client_spec, is_known_capability, resolve_capabilities};
pub use catalog::Catalog;
pub use catalog_cache::{clear_catalog_cache, load_catalog};
pub use schema::{client_spec, defaults, defaults_for_capability};
use std::{
    fs,
    path::{Path, PathBuf},
};

#[derive(Clone, Debug)]
pub struct WebSettings {
    settings_path: PathBuf,
    catalog_path: PathBuf,
}

pub fn sanitize(raw: &Value, catalog: &Catalog) -> Result<Value, Error> {
    let mut settings = defaults();
    for field in schema::FIELDS {
        let value = if raw.has(field.key) {
            raw.get(field.key).clone()
        } else if field.key == "web_upload_url" && raw.has("toss_upload_url") {
            raw.get("toss_upload_url").clone()
        } else {
            field.default()
        };
        set(&mut settings, field.key, field.coerce(&value)?)?;
    }
    for orientation in ["horizontal", "vertical"] {
        let keys = [
            format!("carrot_navi_{orientation}_area_1"),
            format!("carrot_navi_{orientation}_area_2"),
        ];
        let areas =
            catalog.normalize_layout([settings.get(&keys[0]), settings.get(&keys[1])], "live")?;
        for (key, area) in keys.iter().zip(areas) {
            set(&mut settings, key, Value::text(&area))?;
        }
    }
    Ok(settings)
}

/// Drops this port's upload default from a file an earlier build wrote it to,
/// leaving every other key as it is, so carrot-wip uses its own default again.
pub fn forget_stored_upload_default(path: &Path) {
    let Ok(text) = fs::read_to_string(path) else {
        return;
    };
    let Ok(mut value @ Value::Object(_)) = Value::parse(&text) else {
        return;
    };
    let Value::Object(fields) = &mut value else {
        return;
    };
    let before = fields.len();
    fields.retain(|(name, value)| {
        !(name
            .iter()
            .copied()
            .eq("web_upload_url".chars().map(u32::from))
            && value.text_eq(openpilot_web_upload::DEFAULT_WEB_UPLOAD_URL))
    });
    if fields.len() == before {
        return;
    }
    let mut temporary = path.as_os_str().to_os_string();
    temporary.push(".tmp");
    let temporary = PathBuf::from(temporary);
    if crate::state_json::write_json(&temporary, &value).is_ok() {
        let _ = fs::rename(&temporary, path);
    }
}

impl WebSettings {
    pub fn new(settings_path: &Path, catalog_path: &Path) -> Self {
        Self {
            settings_path: settings_path.into(),
            catalog_path: catalog_path.into(),
        }
    }
    pub fn catalog(&self) -> Catalog {
        load_catalog(&self.catalog_path)
    }
    pub fn client_spec(&self) -> Value {
        client_spec(&self.catalog())
    }

    pub fn read(&self) -> Result<Value, Error> {
        let mut raw = match fs::read_to_string(&self.settings_path) {
            Ok(text) => match Value::parse(&text) {
                Ok(value @ Value::Object(_)) => value,
                Ok(
                    Value::Null
                    | Value::Bool(_)
                    | Value::Integer(_)
                    | Value::Float(_)
                    | Value::Text(_)
                    | Value::Array(_),
                )
                | Err(_) => return Ok(defaults()),
            },
            Err(_) => return Ok(defaults()),
        };
        for orientation in ["horizontal", "vertical"] {
            for (suffix, value) in [
                ("mode", "split"),
                ("area_1", "vision"),
                ("area_2", "navigation"),
            ] {
                let key = format!("carrot_navi_{orientation}_{suffix}");
                if !raw.has(&key) {
                    set(&mut raw, &key, Value::text(value))?;
                }
            }
        }
        sanitize(&raw, &self.catalog())
    }

    pub fn write(&self, settings: &Value) -> Result<Value, Error> {
        let clean = sanitize(settings, &self.catalog())?;
        let parent = self.settings_path.parent().unwrap_or_else(|| Path::new(""));
        fs::create_dir_all(parent)?;
        let mut temporary = self.settings_path.as_os_str().to_os_string();
        temporary.push(".tmp");
        let temporary = PathBuf::from(temporary);
        // carrot-wip reads this file too and takes a stored web_upload_url
        // over its own default. Writing this port's default made a device
        // switched back to carrot-wip keep uploading to this port's server;
        // left out, each branch falls back to its own default.
        let mut stored = clean.clone();
        if let Value::Object(fields) = &mut stored {
            fields.retain(|(name, value)| {
                !(name
                    .iter()
                    .copied()
                    .eq("web_upload_url".chars().map(u32::from))
                    && value.text_eq(openpilot_web_upload::DEFAULT_WEB_UPLOAD_URL))
            });
        }
        crate::state_json::write_json(&temporary, &stored)?;
        fs::rename(temporary, &self.settings_path)?;
        Ok(clean)
    }

    pub fn update(&self, updates: &Value) -> Result<Value, Error> {
        let mut current = self.read()?;
        if updates.truth() && !matches!(updates, Value::Object(_)) {
            return Err(Error::Source(format!(
                "'{}' object has no attribute 'items'",
                updates.type_name()
            )));
        }
        if let Value::Object(fields) = updates {
            for (key, value) in fields {
                if let Some(field) = schema::FIELDS
                    .iter()
                    .find(|field| key.iter().copied().eq(field.key.chars().map(u32::from)))
                {
                    set(&mut current, field.key, value.clone())?;
                }
            }
        }
        self.write(&current)
    }
}

#[cfg(test)]
mod upload_default_tests {
    use super::*;

    #[test]
    fn this_ports_upload_default_is_not_stored_for_carrot_wip_to_inherit() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("web_settings.json");
        let settings = WebSettings::new(&path, &root.path().join("catalog.json"));
        let written = settings
            .update(&Value::parse(r#"{"start_page":"tools"}"#).unwrap())
            .unwrap();
        // The caller and later reads still see this port's default.
        assert!(written
            .get("web_upload_url")
            .text_eq(openpilot_web_upload::DEFAULT_WEB_UPLOAD_URL));
        let stored = fs::read_to_string(&path).unwrap();
        assert!(!stored.contains("web_upload_url"), "{stored}");
        assert!(settings
            .read()
            .unwrap()
            .get("web_upload_url")
            .text_eq(openpilot_web_upload::DEFAULT_WEB_UPLOAD_URL));
        // A saved original-project address reads as this port's default and is dropped.
        fs::write(
            &path,
            r#"{"web_upload_url":"https://upload.shind0.synology.me"}"#,
        )
        .unwrap();
        settings
            .update(&Value::parse(r#"{"start_page":"tools"}"#).unwrap())
            .unwrap();
        assert!(!fs::read_to_string(&path)
            .unwrap()
            .contains("web_upload_url"));
        // A file an earlier build wrote keeps its other keys.
        fs::write(
            &path,
            format!(
                r#"{{"carrot_only":1,"web_upload_url":"{}"}}"#,
                openpilot_web_upload::DEFAULT_WEB_UPLOAD_URL
            ),
        )
        .unwrap();
        forget_stored_upload_default(&path);
        let left = fs::read_to_string(&path).unwrap();
        assert!(
            left.contains("carrot_only") && !left.contains("web_upload_url"),
            "{left}"
        );
        // A server the user chose is kept.
        settings
            .update(&Value::parse(r#"{"web_upload_url":"https://example.com"}"#).unwrap())
            .unwrap();
        assert!(fs::read_to_string(&path)
            .unwrap()
            .contains("https://example.com"));
    }
}
