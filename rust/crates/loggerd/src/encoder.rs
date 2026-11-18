use crate::{
    diagnostics,
    media::{Packet, VideoSpec, VideoWriter},
    writer::Logger,
    Error,
};
use openpilot_cereal::log_capnp::{encode_data, event};
use openpilot_logging::{log_site, record::Level};
use openpilot_params::Params;

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

#[derive(Default)]
struct SegmentNumbers {
    offset: Option<i64>,
    last: Option<(u64, u32, i32)>,
}

impl SegmentNumbers {
    fn map(
        &mut self,
        segment: i32,
        encode_id: u32,
        timestamp: u64,
        logger_part: i32,
    ) -> Option<(i64, bool)> {
        let mut restarted = false;
        if let Some((last_time, last_id, last_segment)) = self.last {
            if timestamp < last_time {
                return None;
            }
            // A new encoder starts both counters again. An ordinary packet
            // behind logger_part must never shift the following segments.
            restarted = timestamp > last_time && encode_id < last_id && segment <= last_segment;
        }
        let base = i64::from(segment) - i64::from(logger_part);
        if restarted {
            self.offset = Some(base);
        }
        let offset = *self.offset.get_or_insert(base);
        self.last = Some((timestamp, encode_id, segment));
        Some((i64::from(segment) - offset, restarted))
    }
}

pub struct Encoder {
    pub stream: Stream,
    pub writer: Option<VideoWriter>,
    segments: SegmentNumbers,
    current_segment: i32,
    queue: std::collections::VecDeque<(i64, Vec<u8>)>,
    // rawAudioData from micd is 16 kHz mono; remember any observed source rate.
    pub audio_rate: u32,
    pub include_audio: bool,
    record: bool,
    dropped_frames: u64,
}

#[derive(Default)]
pub struct Outcome {
    pub part: Option<i64>,
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
            segments: SegmentNumbers::default(),
            current_segment: -1,
            queue: Default::default(),
            audio_rate: 16_000,
            include_audio: stream == Stream::Qroad && params.get_bool("RecordAudio")?,
            record,
            dropped_frames: 0,
        })
    }

    pub fn handle(&mut self, logger: &mut Logger, bytes: Vec<u8>) -> Result<Outcome, Error> {
        let message = capnp::serialize::read_message_from_flat_slice(
            &mut bytes.as_slice(),
            Default::default(),
        )?;
        let event = message.get_root::<event::Reader>()?;
        let data = self.stream.data(event)?;
        let index = data.get_idx()?;
        let Some((part, restarted)) = self.segments.map(
            index.get_segment_num(),
            index.get_encode_id(),
            event.get_log_mono_time(),
            logger.part,
        ) else {
            return Ok(Outcome::default());
        };
        if restarted {
            self.queue.clear();
        }
        let mut written = 0;
        match part.cmp(&i64::from(logger.part)) {
            std::cmp::Ordering::Equal => written = self.write(logger, &bytes)?,
            std::cmp::Ordering::Greater => {
                if self.queue.len() < 201 {
                    self.queue.push_back((part, bytes));
                } else {
                    diagnostics::limited(&diagnostics::SEGMENT_QUEUE, log_site!(), || {
                        format!(
                            "{}: dropping frame, queue is too large",
                            self.stream.service()
                        )
                    });
                }
            }
            // The logger already closed this segment.
            std::cmp::Ordering::Less => {}
        }
        Ok(Outcome {
            part: Some(part),
            bytes: written,
        })
    }

    pub fn flush_current(&mut self, logger: &mut Logger) -> Result<usize, Error> {
        let current = i64::from(logger.part);
        let mut written = 0;
        while self.queue.front().is_some_and(|(part, _)| *part <= current) {
            let (part, bytes) = self.queue.pop_front().expect("front exists");
            if part == current {
                written += self.write(logger, &bytes)?;
            }
        }
        Ok(written)
    }

    fn write(&mut self, logger: &mut Logger, bytes: &[u8]) -> Result<usize, Error> {
        let message =
            capnp::serialize::read_message_from_flat_slice(&mut &*bytes, Default::default())?;
        let event = message.get_root::<event::Reader>()?;
        let data = self.stream.data(event)?;
        let index = data.get_idx()?;
        let keyframe = index.get_flags() & 8 != 0;
        let timestamp_us = i64::try_from(index.get_timestamp_eof() / 1000)?;
        if self.current_segment != logger.part {
            if !keyframe {
                self.dropped_frames = self.dropped_frames.wrapping_add(1);
                return Ok(0);
            }
            if self.record {
                self.writer = Some(VideoWriter::open(
                    &logger.path()?.join(self.stream.filename()),
                    VideoSpec {
                        width: data.get_width(),
                        height: data.get_height(),
                        fps: 20,
                        audio_rate: self.include_audio.then_some(self.audio_rate),
                        codec: index.get_type()?,
                    },
                )?);
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
            self.current_segment = logger.part;
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
    use super::SegmentNumbers;

    #[test]
    fn late_packets_do_not_rebase_an_encoder_but_a_restart_does() {
        let mut numbers = SegmentNumbers::default();
        assert_eq!(numbers.map(5, 6000, 100, 5), Some((5, false)));
        assert_eq!(numbers.map(5, 6001, 110, 6), Some((5, false)));
        assert_eq!(numbers.map(6, 6002, 120, 6), Some((6, false)));
        assert_eq!(numbers.map(5, 5999, 90, 6), None);
        assert_eq!(numbers.map(0, 1, 130, 6), Some((6, true)));
        assert_eq!(numbers.map(1, 1201, 140, 7), Some((7, false)));
    }
}
