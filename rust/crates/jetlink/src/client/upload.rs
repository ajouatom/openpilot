use super::{Client, Transport};
use crate::{model_spec::ModelSpec, Deadline, Error};
use serde_json::{json, Value};
use std::{fs::File, io::Read, path::Path, time::Duration};
impl<T: Transport> Client<T> {
    pub(super) fn prepare_legacy_model(&mut self, spec: &ModelSpec) -> Result<Value, Error> {
        self.require_setup()?;
        let app = self
            .app
            .as_ref()
            .ok_or(Error::Contract("App setup missing"))?;
        let (cache, connected) = (app.cache.clone(), app.connected.clone());
        let status = self.status.clone();
        let mut last_progress = 0u64;
        let path = super::model_cache::model_file(
            &cache,
            spec,
            move || {
                if !(connected)() {
                    return Err(Error::Closed);
                }
                Ok(())
            },
            move |stage, fraction| {
                let now = crate::now_ns();
                if now.saturating_sub(last_progress) < 1_000_000_000 {
                    return;
                }
                last_progress = now;
                if let Ok(mut value) = status.lock() {
                    value["state"] = json!("loading");
                    value["preparation"] = json!({"stage":stage,"fraction":fraction,"message":"Preparing verified Cinque v2 for Jetlink"});
                }
            },
        )?;
        self.require_setup()?;
        let deadline = Deadline::after(Duration::from_secs(60))?;
        let sequence = self.send(
            3,
            &serde_json::to_vec(
                &json!({"sha256":spec.sha256,"nbytes":spec.nbytes,"frame_skip":4}),
            )?,
            deadline,
        )?;
        let state: Value =
            serde_json::from_slice(&self.expect_setup(4, sequence, deadline)?.payload)?;
        if state["state"] != "need_upload" {
            return Ok(state);
        }
        self.upload_model(&path, spec, &state)
    }
    /// The App answered need_upload for the model picked on the comma:
    /// download its ONNX (verified against the catalog's sha256 and size) and
    /// upload it. The local copy goes once the App has it, since the App keeps
    /// its own and an 800 MB file per past pick would fill /data.
    pub(super) fn upload_picked_model(
        &mut self,
        pick: &crate::app_pick::AppPick,
    ) -> Result<Value, Error> {
        self.require_setup()?;
        let app = self
            .app
            .as_ref()
            .ok_or(Error::Contract("App setup missing"))?;
        let (cache, connected) = (app.cache.clone(), app.connected.clone());
        let status = self.status.clone();
        let mut last_progress = 0u64;
        let message = format!("Downloading {} for Jetlink", pick.name);
        let path = super::model_cache::model_file_from(
            &cache,
            &pick.spec,
            &pick.url,
            move || {
                if !(connected)() {
                    return Err(Error::Closed);
                }
                Ok(())
            },
            move |stage, fraction| {
                let now = crate::now_ns();
                if now.saturating_sub(last_progress) < 1_000_000_000 {
                    return;
                }
                last_progress = now;
                if let Ok(mut value) = status.lock() {
                    value["state"] = json!("loading");
                    value["preparation"] =
                        json!({"stage":stage,"fraction":fraction,"message":message});
                }
            },
        )?;
        // The download can take minutes: ask again, the App may have the
        // model meanwhile (its owner imported it) or be building it.
        self.require_setup()?;
        let deadline = Deadline::after(Duration::from_secs(60))?;
        let sequence = self.send(
            3,
            &serde_json::to_vec(&json!({"sha256":pick.spec.sha256,"nbytes":pick.spec.nbytes,"frame_skip":pick.spec.frame_skip}))?,
            deadline,
        )?;
        let state: Value =
            serde_json::from_slice(&self.expect_setup(4, sequence, deadline)?.payload)?;
        if state["state"] != "need_upload" {
            return Ok(state);
        }
        let state = self.upload_model(&path, &pick.spec, &state)?;
        if state["state"] != "failed" {
            let _ = std::fs::remove_file(&path);
        }
        Ok(state)
    }
    fn upload_model(
        &mut self,
        path: &Path,
        spec: &ModelSpec,
        state: &Value,
    ) -> Result<Value, Error> {
        let chunk = match state.get("chunk") {
            None | Some(Value::Null) => 4 << 20,
            Some(value) => match value.as_u64() {
                Some(0) => 4 << 20,
                Some(size) if size <= ((16 << 20) - 8) => size as usize,
                _ => return Err(Error::Contract("invalid model upload chunk")),
            },
        };
        self.require_setup()?;
        let mut input = File::open(path)?;
        if input.metadata()?.len() != spec.nbytes {
            return Err(Error::Contract("model upload file size changed"));
        }
        let mut buffer = vec![0; chunk + 8];
        let mut sent = 0u64;
        loop {
            self.require_setup()?;
            let count = input.read(&mut buffer[8..])?;
            if count == 0 {
                break;
            }
            if sent.saturating_add(count as u64) > spec.nbytes {
                return Err(Error::Contract("model upload file grew"));
            }
            buffer[..8].copy_from_slice(&sent.to_le_bytes());
            self.send(
                5,
                &buffer[..count + 8],
                Deadline::after(Duration::from_secs(10))?,
            )?;
            sent += count as u64;
            if let Ok(mut value) = self.status.lock() {
                value["preparation"] = json!({"stage":"upload","fraction":sent as f64/spec.nbytes as f64,"message":"Uploading verified Jetlink model"});
            }
            self.provision_wifi(false)?;
        }
        if sent != spec.nbytes {
            return Err(Error::Contract("model upload file truncated"));
        }
        self.require_setup()?;
        let deadline = Deadline::after(Duration::from_secs(300))?;
        let sequence = self.send(6, b"{}", deadline)?;
        serde_json::from_slice(&self.expect_setup(4, sequence, deadline)?.payload)
            .map_err(Error::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        client::Message,
        wire::{Header, Version},
    };
    use std::collections::VecDeque;
    struct Wire {
        chunks: Vec<(u64, Vec<u8>)>,
        pending: VecDeque<Message>,
        spec: Value,
        done: bool,
    }
    impl Transport for Wire {
        fn send(&mut self, bytes: &[u8], _: Deadline) -> Result<(), Error> {
            let h = Header::decode_version(bytes[..32].try_into().unwrap(), Version::V2)?;
            let p = &bytes[32..32 + h.length as usize];
            match h.kind {
                5 => self.chunks.push((
                    u64::from_le_bytes(p[..8].try_into().unwrap()),
                    p[8..].to_vec(),
                )),
                6 => {
                    self.done = true;
                    let payload = serde_json::to_vec(&json!({"state":"ready","spec":self.spec}))?;
                    self.pending.push_back(Message {
                        header: Header {
                            kind: 4,
                            sequence: h.sequence,
                            flags: 0,
                            length: payload.len() as u32,
                            reserved: 0,
                        },
                        payload,
                    });
                }
                _ => panic!("unexpected upload message"),
            }
            Ok(())
        }
        fn receive(&mut self, _: Deadline) -> Result<Message, Error> {
            self.pending.pop_front().ok_or(Error::Closed)
        }
        fn close(&mut self) {}
    }
    #[test]
    fn actual_wire_upload_uses_offsets_done_response_and_disconnect_cancellation() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("fixture.onnx");
        let data = b"abcdefghijklmnopqrst";
        std::fs::write(&path, data).unwrap();
        let mut spec = ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes()).unwrap();
        spec.nbytes = data.len() as u64;
        let wire = || Wire {
            chunks: Vec::new(),
            pending: VecDeque::new(),
            spec: serde_json::to_value(&spec).unwrap(),
            done: false,
        };
        let mut client = Client::app(wire(), root.path().to_owned());
        assert_eq!(
            client
                .upload_model(&path, &spec, &json!({"chunk":5}))
                .unwrap()["state"],
            "ready"
        );
        assert_eq!(
            client
                .transport
                .chunks
                .iter()
                .map(|(offset, _)| *offset)
                .collect::<Vec<_>>(),
            [0, 5, 10, 15]
        );
        assert_eq!(
            client
                .transport
                .chunks
                .iter()
                .flat_map(|(_, bytes)| bytes.iter().copied())
                .collect::<Vec<_>>(),
            data
        );
        assert!(client.transport.done);
        let checks = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let read = checks.clone();
        let mut client = Client::app(wire(), root.path().to_owned());
        client.set_connected_check(std::sync::Arc::new(move || {
            read.fetch_add(1, std::sync::atomic::Ordering::SeqCst) < 2
        }));
        assert!(client
            .upload_model(&path, &spec, &json!({"chunk":5}))
            .is_err());
        assert_eq!(client.transport.chunks.len(), 1);
        assert!(!client.transport.done);
        for invalid in [json!(-1), json!("5"), json!(32 << 20)] {
            let mut client = Client::app(wire(), root.path().to_owned());
            assert!(client
                .upload_model(&path, &spec, &json!({"chunk":invalid}))
                .is_err());
            assert!(client.transport.chunks.is_empty());
        }
    }
}
