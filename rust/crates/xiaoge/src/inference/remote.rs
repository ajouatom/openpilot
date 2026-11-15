//! `OnnxComputeMode` 3: the Jetlink App's device runs the model. The same
//! prepared input tensor goes to jetlinkd's local socket (`openpilot_jetlink::aux`)
//! and the model's outputs come back to be decoded here exactly as the CPU's
//! are. Any frame the App cannot take (no App, a model still uploading, the
//! tablet hot or busy, a timeout) runs on the comma's CPU instead.
use openpilot_opencv_runtime::{Tensor, TensorView};
use serde_json::{json, Value};
use std::{
    io::{Read, Write},
    os::unix::net::UnixStream,
    path::Path,
    time::{Duration, Instant},
};

/// jetlinkd's socket (`openpilot_jetlink::aux::SOCKET`).
pub const SOCKET: &str = "/dev/shm/carrot-jetlink-aux.sock";
/// After a failed connect: jetlinkd is not running (Jetlink off).
const RETRY: Duration = Duration::from_secs(5);
/// Longest wait for one frame: the input and outputs ride in the gaps after
/// driving replies (~50 ms apart), plus the App's run.
pub const TIMEOUT: Duration = Duration::from_millis(350);
const MAX_REPLY: usize = 16 << 20;

#[derive(Default)]
pub struct Remote {
    stream: Option<UnixStream>,
    retry_at: Option<Instant>,
    /// The App's last answer, for the status page.
    pub last: Value,
    pub frames: u64,
    pub fallbacks: u64,
    pub last_error: String,
}

fn encode(head: &Value, blob: &[u8]) -> Vec<u8> {
    let json = serde_json::to_vec(head).unwrap_or_else(|_| b"{}".to_vec());
    let mut bytes = Vec::with_capacity(8 + json.len() + blob.len());
    bytes.extend(((4 + json.len() + blob.len()) as u32).to_le_bytes());
    bytes.extend((json.len() as u32).to_le_bytes());
    bytes.extend(json);
    bytes.extend_from_slice(blob);
    bytes
}

/// The reply's tensors, in the order the App lists them.
fn tensors(head: &Value, mut blob: &[u8]) -> Result<Vec<Tensor>, String> {
    let outputs = head["outputs"].as_array().ok_or("reply without outputs")?;
    let mut tensors = Vec::with_capacity(outputs.len());
    for output in outputs {
        let shape: Vec<i32> = output["shape"]
            .as_array()
            .ok_or("output without a shape")?
            .iter()
            .map(|dim| {
                dim.as_i64()
                    .and_then(|d| i32::try_from(d).ok())
                    .ok_or("bad dimension")
            })
            .collect::<Result<_, _>>()?;
        let count = shape
            .iter()
            .try_fold(1usize, |n, &d| {
                usize::try_from(d).ok().and_then(|d| n.checked_mul(d))
            })
            .ok_or("bad shape")?;
        let bytes = count.checked_mul(4).ok_or("bad shape")?;
        if blob.len() < bytes {
            return Err(format!(
                "reply has {} bytes for an output of {bytes}",
                blob.len()
            ));
        }
        let values = blob[..bytes]
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();
        blob = &blob[bytes..];
        tensors.push(Tensor::from_parts(shape, values).map_err(|e| e.to_string())?);
    }
    if !blob.is_empty() {
        return Err(format!("{} bytes past the outputs", blob.len()));
    }
    Ok(tensors)
}

impl Remote {
    fn connect(&mut self) -> Result<&mut UnixStream, String> {
        if self.stream.is_none() {
            if self.retry_at.is_some_and(|at| Instant::now() < at) {
                return Err("jetlinkd is not running".into());
            }
            match UnixStream::connect(SOCKET) {
                Ok(stream) => {
                    self.retry_at = None;
                    self.stream = Some(stream);
                }
                Err(error) => {
                    self.retry_at = Some(Instant::now() + RETRY);
                    return Err(format!("jetlinkd is not running: {error}"));
                }
            }
        }
        self.stream.as_mut().ok_or_else(|| "no socket".into())
    }

    fn exchange(&mut self, request: &[u8], timeout: Duration) -> Result<Vec<u8>, String> {
        let stream = self.connect()?;
        let io = (|| -> std::io::Result<Vec<u8>> {
            stream.set_write_timeout(Some(timeout))?;
            // jetlinkd answers within `timeout`; the margin covers its own work.
            stream.set_read_timeout(Some(timeout + Duration::from_millis(250)))?;
            stream.write_all(request)?;
            let mut size = [0; 4];
            stream.read_exact(&mut size)?;
            let size = u32::from_le_bytes(size) as usize;
            if !(4..=MAX_REPLY).contains(&size) {
                return Err(std::io::Error::other("reply size"));
            }
            let mut reply = vec![0; size];
            stream.read_exact(&mut reply)?;
            Ok(reply)
        })();
        io.map_err(|error| {
            // A half-read reply leaves the stream out of step: start again.
            self.stream = None;
            format!("jetlinkd socket: {error}")
        })
    }

    /// Runs `model` on the App, or says why not.
    pub fn run(
        &mut self,
        model: &Path,
        input: TensorView<'_>,
        names: &[String],
    ) -> Result<Vec<Tensor>, String> {
        let result = (|| {
            let mut blob = Vec::with_capacity(input.values().len() * 4);
            for value in input.values() {
                blob.extend(value.to_le_bytes());
            }
            let head = json!({
                "model": model.to_string_lossy(),
                "outputs": names,
                "shape": input.shape(),
                "timeout_ms": TIMEOUT.as_millis() as u64,
            });
            let reply = self.exchange(&encode(&head, &blob), TIMEOUT)?;
            let length = reply
                .get(..4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize)
                .ok_or("short reply")?;
            let head: Value = reply
                .get(4..4 + length)
                .and_then(|json| serde_json::from_slice(json).ok())
                .ok_or("bad reply")?;
            if head["ok"] != true {
                return Err(format!(
                    "{}: {}",
                    head["error"].as_str().unwrap_or("failed"),
                    head["detail"].as_str().unwrap_or_default()
                ));
            }
            let outputs = tensors(&head, &reply[4 + length..])?;
            self.last = json!({
                "device": head["device"], "run_us": head["run_us"],
                "queue_us": head["queue_us"], "round_ms": head["round_ms"],
            });
            Ok(outputs)
        })();
        match &result {
            Ok(_) => {
                self.frames += 1;
                self.last_error.clear();
            }
            Err(error) => {
                self.fallbacks += 1;
                self.last_error = error.chars().take(200).collect();
            }
        }
        result
    }

    pub fn device(&self) -> Option<&str> {
        self.last["device"].as_str()
    }

    pub fn status(&self) -> Value {
        json!({"frames": self.frames, "fallbacks": self.fallbacks,
            "last": self.last, "last_error": self.last_error})
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reply_tensors_follow_their_shapes() {
        let head =
            json!({"outputs": [{"name": "a", "shape": [1, 2]}, {"name": "b", "shape": [1]}]});
        let mut blob = Vec::new();
        for v in [1.0f32, 2.0, 3.0] {
            blob.extend(v.to_le_bytes());
        }
        let out = tensors(&head, &blob).unwrap();
        assert_eq!(out[0].view().shape(), [1, 2]);
        assert_eq!(out[0].view().values(), [1.0, 2.0]);
        assert_eq!(out[1].view().values(), [3.0]);
        assert!(tensors(&head, &blob[..8]).is_err());
        let mut long = blob.clone();
        long.extend([0; 4]);
        assert!(tensors(&head, &long).is_err());
    }

    #[test]
    fn request_framing_counts_the_json_and_the_blob() {
        let bytes = encode(&json!({"a": 1}), &[9, 9]);
        let total = u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize;
        assert_eq!(total, bytes.len() - 4);
        let json = u32::from_le_bytes(bytes[4..8].try_into().unwrap()) as usize;
        assert_eq!(&bytes[8..8 + json], br#"{"a":1}"#);
        assert_eq!(&bytes[8 + json..], [9, 9]);
    }
}
