use super::{config::Config, parameters};
use crate::Error;
use openpilot_params::Params;
use openpilot_web_upload::{Environment, Fields, Response, SessionMode, TmuxUpload, Value};
use std::{
    fs,
    io::{Read, Seek, SeekFrom},
    process::{Command, Stdio},
};
#[path = "upload_build.rs"]
mod build;
#[path = "upload_discord.rs"]
mod discord;
#[path = "upload_snapshot.rs"]
mod snapshot;

pub struct Upload<'a> {
    pub config: &'a Config,
    pub params: &'a Params,
}
impl Upload<'_> {
    pub fn capture(&self) -> bool {
        let result = (|| {
            let path = self.config.data_root.join("media/tmux.log");
            match fs::remove_file(&path) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => return Err(e.into()),
            }
            let output = fs::File::create(&path)?;
            let status = Command::new("tmux")
                .args(["capture-pane", "-pq", "-S-1000"])
                .stdout(output)
                .stderr(Stdio::piped())
                .output()?
                .status;
            if !status.success() {
                return Err(Error::Contract("tmux capture failed"));
            }
            // Evidence only: a report without it is still a report.
            let summary = build::summary(
                &self.config.data_root.join("openpilot-build"),
                std::time::SystemTime::now(),
            );
            if !summary.is_empty() {
                use std::io::Write;
                if let Ok(mut file) = fs::OpenOptions::new().append(true).open(&path) {
                    let _ = file.write_all(summary.as_bytes());
                }
            }
            let _ = snapshot::backup(&self.config.data_root);
            Ok::<_, Error>(())
        })();
        if let Err(e) = result {
            eprintln!("carrot_man TMUX creation error: {e}");
            false
        } else {
            true
        }
    }
    pub fn save_toggles(&self) {
        if let Err(e) = snapshot::toggles(self.params, &self.config.data_root) {
            eprintln!("carrot_man save_toggle_values: {e}");
        }
    }
    fn payload(&self, why: &str) -> Fields {
        let mut payload = Fields::new();
        payload.insert("tmux_why".into(), Value::Text(why.into()));
        for (field, key) in [
            ("car_name", "CarName"),
            ("git_branch", "GitBranch"),
            ("github_id", "GithubUsername"),
            ("git_remote", "GitRemote"),
            ("git_commit", "GitCommit"),
            ("git_commit_date", "GitCommitDate"),
            ("dongle_id", "DongleId"),
            ("device_serial", "HardwareSerial"),
        ] {
            payload.insert(
                field.into(),
                Value::Text(
                    self.params
                        .get(key)
                        .ok()
                        .flatten()
                        .map(|b| snapshot::utf8_ignore(&b))
                        .unwrap_or_default(),
                ),
            );
        }
        let ip = nix::ifaddrs::getifaddrs().ok().and_then(|mut interfaces| {
            interfaces.find_map(|i| {
                let ip = i.address.as_ref()?.as_sockaddr_in()?.ip();
                (i.interface_name == "wlan0" && snapshot::private_ip(ip)).then(|| ip.to_string())
            })
        });
        payload.insert("local_ip".into(), ip.map_or(Value::Null, Value::Text));
        payload
    }
    fn post(
        &self,
        target: &openpilot_web_upload::Target,
        payload: &Fields,
        settings: bool,
    ) -> Result<Response, Error> {
        if settings {
            self.save_toggles();
        }
        let tmux = self.config.data_root.join("media/tmux.log");
        let toggles = self.config.data_root.join("toggle_values.json");
        Ok(TmuxUpload {
            target,
            payload,
            tmux_path: &tmux,
            settings_path: settings.then_some(toggles.as_path()),
        }
        .post()?)
    }
    pub fn web(&self, why: &str, settings: bool) -> Option<Response> {
        let result = (|| {
            let private = self
                .config
                .web_settings
                .with_file_name("private_upload.json");
            if let Some(webdav) = openpilot_web_upload::WebDav::load(&private)
                .map_err(openpilot_web_upload::Error::from)?
            {
                if settings {
                    snapshot::toggles(self.params, &self.config.data_root)?;
                }
                return Ok(webdav.upload_tmux(
                    &self.payload(why),
                    &self.config.data_root.join("media/tmux.log"),
                    settings
                        .then_some(self.config.data_root.join("toggle_values.json"))
                        .as_deref(),
                )?);
            }
            let values = snapshot::web_settings(&self.config.web_settings);
            let environment = Environment::for_runtime()
                .map_err(|_| Error::Contract("upload environment encoding"))?;
            let payload = self.payload(why);
            let (base, token) = openpilot_web_upload::web_settings(&values, &environment)?;
            let token = if token.is_empty() {
                openpilot_web_upload::create_session(&base, &payload, SessionMode::Sync)?
            } else {
                token
            };
            self.post(
                &openpilot_web_upload::tmux_target(&values, &environment, &token)?,
                &payload,
                settings,
            )
        })();
        match result {
            Ok(response) => Some(response),
            Err(error) => {
                eprintln!("carrot_man web tmux sending error: {error}");
                None
            }
        }
    }
    pub fn carrot_logs(&self, why: &str, settings: bool) -> Option<Response> {
        let result = (|| {
            let environment = Environment::for_runtime()
                .map_err(|_| Error::Contract("upload environment encoding"))?;
            if !openpilot_web_upload::carrot_logs_configured(&environment) {
                return Ok(None);
            }
            self.post(
                &openpilot_web_upload::carrot_logs_target(&environment)?,
                &self.payload(why),
                settings,
            )
            .map(Some)
        })();
        match result {
            Ok(response) => response,
            Err(error) => {
                eprintln!("carrot_man carrot_logs tmux sending error: {error}");
                None
            }
        }
    }
    pub fn discord(
        &self,
        why: &str,
        web_ok: bool,
        response: Option<&Response>,
        settings: bool,
    ) -> bool {
        discord::send(self, why, web_ok, response, settings)
    }
}
pub fn ok(response: Option<&Response>) -> bool {
    response.is_some_and(|r| r.status < 400)
}

fn attachment(path: &std::path::Path) -> Result<Vec<u8>, Error> {
    let mut file = fs::File::open(path)?;
    let size = file.metadata()?.len();
    let limit = 8 * 1024 * 1024_u64;
    if size <= limit {
        return Ok(fs::read(path)?);
    }
    let mut output = Vec::new();
    file.by_ref().take(limit / 2).read_to_end(&mut output)?;
    output.extend_from_slice(b"\n\n===== DISCORD TMUX TRUNCATED =====\n\n");
    file.seek(SeekFrom::Start(size.saturating_sub(limit / 2)))?;
    file.take(limit / 2).read_to_end(&mut output)?;
    Ok(output)
}

#[cfg(test)]
#[path = "upload_tests.rs"]
mod tests;
