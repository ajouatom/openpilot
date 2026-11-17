use crate::{
    diagnostics,
    media::{Packet, VideoSpec, VideoWriter},
    writer::Logger,
    Error,
};
use openpilot_cereal::log_capnp::{encode_data, event};
use openpilot_logging::{log_site, record::Level};
use openpilot_params::Params;

/// Audio is recorded when `RecordAudio` asks for it, never on a C3X Lite:
/// it has no microphone, and qcamera would hold its frames for audio that
/// never comes.
pub fn record_audio(params: &Params) -> Result<bool, Error> {
    Ok(audio_wanted(
        params.get_bool("RecordAudio")?,
        params.get_bool("HardwareC3xLite")?,
    ))
}

fn audio_wanted(record_audio: bool, c3x_lite: bool) -> bool {
    record_audio && !c3x_lite
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stream {
    Road,
    Wide,
    Driver,
    Qroad,
}

impl Stream {
    fn service(self) -> &'static str {
        match self {
            Self::Road => "roadEncodeData",
            Self::Wide => "wideRoadEncodeData",
            Self::Driver => "driverEncodeData",
            Self::Qroad => "qRoadEncodeData",
        }
    }

    /// Position in `Rotation::stream_seen_ns`.
    pub fn index(self) -> usize {
        match self {
            Self::Road => 0,
            Self::Wide => 1,
            Self::Driver => 2,
            Self::Qroad => 3,
        }
    }

    pub fn from_service(name: &str) -> Option<Self> {
        match name {
            "roadEncodeData" => Some(Self::Road),
            "wideRoadEncodeData" => Some(Self::Wide),
            "driverEncodeData" => Some(Self::Driver),
            "qRoadEncodeData" => Some(Self::Qroad),
            _ => None,
        }
    }

    fn data(self, event: event::Reader<'_>) -> Result<encode_data::Reader<'_>, Error> {
        match (self, event.which()?) {
            (Self::Road, event::RoadEncodeData(data))
            | (Self::Wide, event::WideRoadEncodeData(data))
            | (Self::Driver, event::DriverEncodeData(data))
            | (Self::Qroad, event::QRoadEncodeData(data)) => Ok(data?),
            _ => Err(Error::Invalid("encoder event does not match service")),
        }
    }

    fn filename(self) -> &'static str {
        match self {
            Self::Road => "fcamera.hevc",
            Self::Wide => "ecamera.hevc",
            Self::Driver => "dcamera.hevc",
            Self::Qroad => "qcamera.ts",
        }
    }
}

pub struct Encoder {
    pub stream: Stream,
    pub writer: Option<VideoWriter>,
    offset: Option<i64>,
    current_segment: i32,
    queue: Vec<Vec<u8>>,
    recording: bool,
    marked_ready: bool,
    pub audio_initialized: bool,
    pub include_audio: bool,
    record: bool,
    dropped_frames: u64,
}

#[derive(Default)]
pub struct Outcome {
    pub ready_part: Option<i64>,
    pub bytes: usize,
}

impl Encoder {
    pub fn new(stream: Stream, params: &Params) -> Result<Self, Error> {
        let road = params.get("RecordRoadCam")?.unwrap_or_default();
        let road = String::from_utf8_lossy(&road);
        let road = if road.is_empty() {
            0
        } else {
            let value = road.trim_start_matches(|character: char| character.is_ascii_whitespace());
            let sign = usize::from(value.starts_with(['+', '-']));
            let end = sign + value[sign..].bytes().take_while(u8::is_ascii_digit).count();
            value[..end]
                .parse::<i32>()
                .map_err(|_| Error::Invalid("invalid RecordRoadCam integer"))?
        };
        let record = match stream {
            Stream::Road => road > 0,
            Stream::Wide => road > 1,
            Stream::Driver => params.get_bool("RecordFront")?,
            Stream::Qroad => true,
        };
        Ok(Self {
            stream,
            writer: None,
            offset: None,
            current_segment: -1,
            queue: Vec::new(),
            recording: false,
            marked_ready: false,
            audio_initialized: false,
            include_audio: stream == Stream::Qroad && record_audio(params)?,
            record,
            dropped_frames: 0,
        })
    }

    pub fn handle(
        &mut self,
        logger: &mut Logger,
        bytes: Vec<u8>,
        ready: usize,
    ) -> Result<Outcome, Error> {
        let message = capnp::serialize::read_message_from_flat_slice(
            &mut bytes.as_slice(),
            Default::default(),
        )?;
        let event = message.get_root::<event::Reader>()?;
        let data = self.stream.data(event)?;
        let index = data.get_idx()?;
        let offset = match self.offset {
            Some(offset) => offset,
            None => {
                let offset = i64::from(index.get_segment_num());
                self.offset = Some(offset);
                diagnostics::emit(
                    log_site!(),
                    Level::Debug,
                    format!("{}: has encoderd offset {offset}", self.stream.service()),
                );
                offset
            }
        };
        let part = i64::from(index.get_segment_num()) - offset;
        match part.cmp(&i64::from(logger.part)) {
            std::cmp::Ordering::Equal => {
                if self.current_segment != logger.part {
                    if self.record {
                        let next = VideoWriter::open(
                            &logger.path()?.join(self.stream.filename()),
                            VideoSpec {
                                width: data.get_width(),
                                height: data.get_height(),
                                fps: 20,
                                codec: index.get_type()?,
                            },
                        )?;
                        if let Some(writer) = self.writer.replace(next) {
                            writer.close()?;
                        }
                        self.recording = false;
                        self.audio_initialized = false;
                    }
                    self.current_segment = logger.part;
                    self.marked_ready = false;
                }
                let mut written = 0;
                if self.audio_initialized || !self.include_audio {
                    for queued in std::mem::take(&mut self.queue) {
                        written += self.write(logger, &queued)?;
                    }
                    written += self.write(logger, &bytes)?;
                } else {
                    self.enqueue(bytes, true);
                }
                Ok(Outcome {
                    ready_part: None,
                    bytes: written,
                })
            }
            std::cmp::Ordering::Greater => {
                let newly_ready = !self.marked_ready;
                self.marked_ready = true;
                if newly_ready {
                    diagnostics::emit(
                        log_site!(),
                        Level::Debug,
                        format!(
                            "rotate {} -> {part} ready {}/4 for {}",
                            logger.part,
                            ready + 1,
                            self.stream.service()
                        ),
                    );
                }
                self.enqueue(bytes, false);
                Ok(Outcome {
                    ready_part: newly_ready.then_some(part),
                    bytes: 0,
                })
            }
            std::cmp::Ordering::Less => {
                diagnostics::emit(log_site!(), Level::Error, format!(
                    "{}: encoderd packet has a older segment!!! idx.getSegmentNum():{} s->logger.segment():{} re.encoderd_segment_offset:{offset}",
                    self.stream.service(), index.get_segment_num(), logger.part));
                self.offset = Some(-i64::from(logger.part));
                Ok(Outcome::default())
            }
        }
    }

    fn enqueue(&mut self, bytes: Vec<u8>, waiting_audio: bool) {
        if self.queue.len() > 200 {
            if waiting_audio {
                diagnostics::limited(&diagnostics::AUDIO_QUEUE, log_site!(), || {
                    format!(
                        "{}: dropping frame waiting for audio initialization, queue is too large",
                        self.stream.service()
                    )
                });
            } else {
                diagnostics::limited(&diagnostics::SEGMENT_QUEUE, log_site!(), || {
                    format!(
                        "{}: dropping frame, queue is too large",
                        self.stream.service()
                    )
                });
            }
        } else {
            self.queue.push(bytes);
        }
    }

    fn write(&mut self, logger: &mut Logger, bytes: &[u8]) -> Result<usize, Error> {
        let message =
            capnp::serialize::read_message_from_flat_slice(&mut &*bytes, Default::default())?;
        let event = message.get_root::<event::Reader>()?;
        let data = self.stream.data(event)?;
        let index = data.get_idx()?;
        let keyframe = index.get_flags() & 8 != 0;
        let timestamp_us = i64::try_from(index.get_timestamp_eof() / 1000)?;
        if !self.recording {
            if !keyframe {
                self.dropped_frames = self.dropped_frames.wrapping_add(1);
                return Ok(0);
            }
            if self.dropped_frames > 0 {
                diagnostics::emit(
                    log_site!(),
                    Level::Warning,
                    format!(
                        "{}: dropped {} non iframe packets before init",
                        self.stream.service(),
                        self.dropped_frames
                    ),
                );
                self.dropped_frames = 0;
            }
            if let Some(writer) = &mut self.writer {
                writer.write(Packet {
                    data: data.get_header()?,
                    timestamp_us,
                    configuration: true,
                    keyframe: false,
                })?;
            }
            self.recording = true;
        }
        if let Some(writer) = &mut self.writer {
            writer.write(Packet {
                data: data.get_data()?,
                timestamp_us,
                configuration: false,
                keyframe,
            })?;
        }
        let mut message = capnp::message::Builder::new_default();
        let mut output = message.init_root::<event::Builder>();
        output.set_valid(event.get_valid());
        output.set_log_mono_time(event.get_log_mono_time());
        match self.stream {
            Stream::Road => output.set_road_encode_idx(index)?,
            Stream::Wide => output.set_wide_road_encode_idx(index)?,
            Stream::Driver => output.set_driver_encode_idx(index)?,
            Stream::Qroad => output.set_q_road_encode_idx(index)?,
        }
        let bytes = capnp::serialize::write_message_to_words(&message);
        logger.write(&bytes, true)?;
        Ok(bytes.len())
    }

    pub fn close(&mut self) -> Result<(), Error> {
        if let Some(writer) = self.writer.take() {
            writer.close()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_lite_never_records_or_waits_for_audio() {
        assert!(super::audio_wanted(true, false));
        assert!(!super::audio_wanted(true, true));
        assert!(!super::audio_wanted(false, false));
    }
}
