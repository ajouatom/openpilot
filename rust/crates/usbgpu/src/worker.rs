use crate::Error;
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{Read, Write},
    os::unix::fs::FileExt,
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Input {
    pub name: String,
    pub shape: Vec<usize>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Metadata {
    pub model_sha256: String,
    pub checkpoint: String,
    pub inputs: Vec<Input>,
    pub output_count: usize,
    pub output_slices: BTreeMap<String, [Option<usize>; 3]>,
}
#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct View {
    pub offset: usize,
    pub shape: Vec<usize>,
    pub dtype: String,
}
impl View {
    fn bytes(&self) -> Result<usize, Error> {
        self.shape
            .iter()
            .try_fold(if self.dtype == "uint8" { 1usize } else { 4 }, |n, d| {
                n.checked_mul(*d)
                    .ok_or(Error::Contract("worker shape overflow"))
            })
    }
}
#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Info {
    pub size: usize,
    pub input_bytes: usize,
    pub output_count: usize,
    pub layout: BTreeMap<String, View>,
    pub input_shapes: BTreeMap<String, Vec<usize>>,
    pub output_slices: BTreeMap<String, [Option<usize>; 3]>,
    pub checkpoint: String,
    pub frame_size: usize,
}
impl Info {
    pub fn local(metadata: Metadata, camera: [u32; 2]) -> Result<Self, Error> {
        let mut info = Self::new(metadata, camera)?;
        let [width, height] = camera.map(|value| {
            usize::try_from(value).map_err(|_| Error::Contract("local camera size overflow"))
        });
        let (width, height) = (width?, height?);
        let stride = width.next_multiple_of(128);
        let y_height = height.next_multiple_of(32);
        let uv_height = (height / 2).next_multiple_of(16);
        let planes = stride * y_height + stride * uv_height + 4096;
        let full = (planes + (16 * 1024).max(8 * stride)).next_multiple_of(4096);
        let full = (full + width.next_multiple_of(512) * 512).next_multiple_of(4096);
        let start = info
            .layout
            .get("img")
            .ok_or(Error::Contract("local image layout missing"))?
            .offset;
        for (index, name) in ["img", "big_img"].into_iter().enumerate() {
            let view = info
                .layout
                .get_mut(name)
                .ok_or(Error::Contract("local image layout missing"))?;
            view.offset = start + index * full;
            view.shape = vec![full];
            info.input_shapes.insert(name.into(), vec![full]);
        }
        info.frame_size = full;
        info.input_bytes = start + 2 * full;
        info.size = info.input_bytes + info.output_count * 4;
        Ok(info)
    }
    pub fn new(metadata: Metadata, camera: [u32; 2]) -> Result<Self, Error> {
        if ![[1344, 760], [1928, 1208]].contains(&camera)
            || metadata.checkpoint.is_empty()
            || metadata.output_count == 0
            || metadata.output_count > 1_000_000
            || metadata.output_slices.is_empty()
            || metadata.output_slices.len() > 256
            || !matches!(metadata.inputs.len(), 3 | 5)
        {
            return Err(Error::Contract("unsupported worker metadata"));
        }
        for section in metadata.output_slices.values() {
            if !matches!(section, [Some(start), Some(stop), None | Some(1)] if start < stop && *stop <= metadata.output_count)
            {
                return Err(Error::Contract("invalid worker output slice"));
            }
        }
        let [width, height] = camera;
        let frame_size = (width.next_multiple_of(128)
            * (height.next_multiple_of(32) + (height / 2).next_multiple_of(16)))
            as usize;
        let mut layout = BTreeMap::from([
            (
                "tfm".into(),
                View {
                    offset: 0,
                    shape: vec![3, 3],
                    dtype: "float32".into(),
                },
            ),
            (
                "big_tfm".into(),
                View {
                    offset: 36,
                    shape: vec![3, 3],
                    dtype: "float32".into(),
                },
            ),
        ]);
        let mut offset = 128;
        for input in metadata.inputs {
            let size: usize = match input.name.as_str() {
                "desire" if input.shape == [8] => 32,
                "traffic_convention" | "action_t" if input.shape == [1, 2] => 8,
                "nav_map" if input.shape == [1, 1, 256, 256] => 256 * 256 * 4,
                "navigation" if input.shape == [1, 47] => 47 * 4,
                _ => return Err(Error::Contract("unsupported worker input shape")),
            };
            if layout
                .insert(
                    input.name,
                    View {
                        offset,
                        shape: input.shape,
                        dtype: "float32".into(),
                    },
                )
                .is_some()
            {
                return Err(Error::Contract("duplicate worker input"));
            }
            offset += size.next_multiple_of(128);
        }
        if !["desire", "traffic_convention", "action_t"]
            .iter()
            .all(|name| layout.contains_key(*name))
            || layout.contains_key("navigation") != layout.contains_key("nav_map")
        {
            return Err(Error::Contract("incomplete worker input layout"));
        }
        let mut input_shapes = BTreeMap::new();
        for name in ["img", "big_img"] {
            layout.insert(
                name.into(),
                View {
                    offset,
                    shape: vec![frame_size],
                    dtype: "uint8".into(),
                },
            );
            input_shapes.insert(name.into(), vec![frame_size]);
            offset += frame_size;
        }
        Ok(Self {
            size: offset + metadata.output_count * 4,
            input_bytes: offset,
            output_count: metadata.output_count,
            layout,
            input_shapes,
            output_slices: metadata.output_slices,
            checkpoint: metadata.checkpoint,
            frame_size,
        })
    }
    pub fn input<'a>(&self, bytes: &'a [u8], name: &str) -> Result<&'a [u8], Error> {
        let view = self
            .layout
            .get(name)
            .ok_or(Error::Contract("worker view missing"))?;
        bytes
            .get(view.offset..view.offset + view.bytes()?)
            .ok_or(Error::Contract("worker input outside shared buffer"))
    }
}
pub trait Runtime {
    fn reset(&mut self) -> Result<(), Error> {
        Err(Error::Contract(
            "model worker does not support recurrent reset",
        ))
    }
    fn run(&mut self, packed: &[u8], info: &Info, output: &mut [u8]) -> Result<(), Error>;
    fn prepare(&mut self, packed: &[u8], info: &Info, output: &mut [u8]) -> Result<bool, Error> {
        self.run(packed, info, output)?;
        Ok(true)
    }
}
pub fn watch_parent() -> bool {
    watch_parent_progress(None)
}
/// Watch the parent process, independent of the lifetime of its loading thread.
pub fn watch_parent_progress(progress: Option<std::path::PathBuf>) -> bool {
    let parent = rustix::process::getppid();
    if parent.is_none_or(|pid| pid.as_raw_nonzero().get() == 1) {
        return false;
    }
    std::thread::spawn(move || {
        let mut watchdog = crate::worker_progress::Watchdog::new(progress);
        while rustix::process::getppid() == parent {
            watchdog.tick();
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        std::process::exit(1);
    });
    true
}
pub fn serve(
    runtime: &mut impl Runtime,
    file: &File,
    info: &Info,
    control_in: &mut impl Read,
    control_out: &mut impl Write,
) -> Result<(), Error> {
    crate::worker_progress::mark("bind_buffers");
    file.set_len(info.size as u64)?;
    let mut packed = vec![0; info.input_bytes];
    let mut output = vec![0; info.output_count * 4];
    file.write_all_at(&packed, 0)?;
    serde_json::to_writer(&mut *control_out, info)?;
    control_out.write_all(b"\n")?;
    control_out.flush()?;
    crate::worker_progress::mark("idle");
    let mut command = [0];
    while control_in.read(&mut command)? != 0 {
        match command[0] {
            b'q' => break,
            b'z' => {
                runtime.reset()?;
                packed.fill(0);
                output.fill(0);
                file.write_all_at(&packed, 0)?;
                file.write_all_at(&output, info.input_bytes as u64)?;
                control_out.write_all(b"1\n")?;
                control_out.flush()?;
            }
            b'r' | b'p' => {
                crate::worker_progress::next_frame();
                file.read_exact_at(&mut packed, 0)?;
                let publish = if command[0] == b'p' {
                    runtime.prepare(&packed, info, &mut output)?
                } else {
                    runtime.run(&packed, info, &mut output)?;
                    true
                };
                if !publish {
                    crate::worker_progress::mark("idle");
                    control_out.write_all(b"0\n")?;
                    control_out.flush()?;
                    continue;
                }
                if output
                    .chunks_exact(4)
                    .any(|value| !f32::from_le_bytes(value.try_into().unwrap()).is_finite())
                {
                    return Err(Error::Contract("invalid precompiled model output"));
                }
                crate::worker_progress::mark("publish");
                file.write_all_at(&output, info.input_bytes as u64)?;
                control_out.write_all(b"1\n")?;
                control_out.flush()?;
                crate::worker_progress::mark("idle");
            }
            _ => return Err(Error::Contract("invalid model worker command")),
        }
    }
    Ok(())
}
/// Preserve the pinned source's terminal exception identity across process IPC.
pub fn diagnostic(error: &Error) -> String {
    match error.worker_cause() {
        Error::UsbApi {
            operation: "HCQ USB control transfer failed",
            message,
            ..
        } => format!("libusb_control_transfer: {message}"),
        Error::UsbApi {
            operation: "HCQ USB bulk transfer failed",
            message,
            ..
        } => format!("libusb_bulk_transfer: {message}"),
        Error::UsbShortTransfer {
            operation: "HCQ USB control short transfer" | "HCQ USB bulk short transfer",
            ..
        } => "RuntimeError: AMD failed with -1".to_owned(),
        Error::Timeout {
            operation,
            milliseconds,
            last: Some(last),
            expected,
        } if operation == "BL not ready" => format!(
            "TimeoutError: BL not ready. Timed out after {milliseconds} ms, condition not met: {last} != {expected}"
        ),
        Error::Contract("invalid precompiled model output") => {
            "ValueError: invalid precompiled model output".to_owned()
        }
        _ => error.to_string(),
    }
}

pub fn report_error(error: &Error, control: &mut impl Write) -> std::io::Result<()> {
    let text = diagnostic(error);
    let mut start = text.len().saturating_sub(16384);
    while !text.is_char_boundary(start) {
        start += 1;
    }
    control.write_all(b"ERROR ")?;
    serde_json::to_writer(&mut *control, &text[start..])?;
    control.write_all(b"\n")?;
    control.flush()
}
