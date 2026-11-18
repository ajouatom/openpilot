use crate::diagnostics;
use crate::{audio::Audio, codec, raw_file::RawFile, Error};
use ffmpeg_next::{self as av, codec::Id, format::context::Output, Rescale};
use openpilot_cereal::log_capnp::encode_index::Type;
use openpilot_logging::{log_site, record::Level};
use std::{
    fs::{self, OpenOptions},
    os::unix::fs::OpenOptionsExt,
    path::{Path, PathBuf},
};

#[derive(Clone, Copy)]
pub struct VideoSpec {
    pub width: u32,
    pub height: u32,
    pub fps: i32,
    pub audio_rate: Option<u32>,
    pub codec: Type,
}

pub struct Packet<'a> {
    pub data: &'a [u8],
    pub timestamp_us: i64,
    pub configuration: bool,
    pub keyframe: bool,
}

pub struct Muxer {
    pub output: Output,
    context: av::codec::encoder::video::Video,
    pub header_written: bool,
    audio: Option<Audio>,
}

enum Destination {
    Raw(RawFile),
    Mux(Muxer),
}

pub struct VideoWriter {
    destination: Destination,
    lock: PathBuf,
}

impl VideoWriter {
    pub fn open(path: &Path, spec: VideoSpec) -> Result<Self, Error> {
        let mut lock = path.as_os_str().to_owned();
        lock.push(".lock");
        let lock = PathBuf::from(lock);
        OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o664)
            .open(&lock)?;
        diagnostics::emit(
            log_site!(),
            Level::Debug,
            format!(
                "encoder_open {} remuxing:{}",
                path.display(),
                u8::from(spec.codec != Type::FullHEVC)
            ),
        );
        let destination = match spec.codec {
            Type::FullHEVC => Destination::Raw(RawFile::create(path)?),
            Type::BigBoxLossless
            | Type::QcameraH264
            | Type::LivestreamH264
            | Type::BigBoxHEVCDEPRECATED
            | Type::ChffrAndroidH264DEPRECATED
            | Type::FullLosslessClipDEPRECATED
            | Type::FrontDEPRECATED => {
                if spec.width == 0 || spec.height == 0 || spec.fps <= 0 {
                    return Err(Error::Invalid("invalid encoded video dimensions/rate"));
                }
                av::init()?;
                let lossless = spec.codec == Type::BigBoxLossless;
                let mut output = if lossless {
                    av::format::output_as(path, "matroska")?
                } else {
                    av::format::output(path)?
                };
                let id = if lossless { Id::FFVHUFF } else { Id::H264 };
                let encoder = if lossless {
                    Some(av::encoder::find(id).ok_or(av::Error::EncoderNotFound)?)
                } else {
                    None
                };
                let mut context = codec::video_context(encoder)?;
                context.set_width(spec.width);
                context.set_height(spec.height);
                context.set_format(av::format::Pixel::YUV420P);
                context.set_time_base((1, spec.fps));
                let context = if lossless {
                    // FFVHUFF derives indispensable extradata when opened, as the original writer does.
                    context.open_as(encoder)?.0
                } else {
                    context
                };
                output.add_stream(encoder)?;
                // Declare the audio stream before the video header. Video can start
                // at its first keyframe even if the microphone is late or absent.
                let audio = spec
                    .audio_rate
                    .and_then(|rate| match Audio::new(&mut output, rate) {
                        Ok(audio) => Some(audio),
                        Err(error) => {
                            diagnostics::emit(
                                log_site!(),
                                Level::Warning,
                                format!("recording video without audio: {error}"),
                            );
                            None
                        }
                    });
                Destination::Mux(Muxer {
                    output,
                    context,
                    header_written: false,
                    audio,
                })
            }
        };
        Ok(Self { destination, lock })
    }

    pub fn write(&mut self, packet: Packet<'_>) -> Result<(), Error> {
        match &mut self.destination {
            Destination::Raw(file) => {
                if let Err(error) = file.write(packet.data) {
                    diagnostics::emit(
                        log_site!(),
                        Level::Error,
                        format!(
                            "failed to write file.errno={}",
                            error.raw_os_error().unwrap_or(0)
                        ),
                    );
                }
            }
            Destination::Mux(muxer) => {
                if packet.configuration {
                    codec::set_extradata(&mut muxer.context, packet.data)?;
                    muxer
                        .output
                        .stream_mut(0)
                        .ok_or(Error::Invalid("missing video stream"))?
                        .set_parameters(&muxer.context);
                    if muxer.audio.is_some() {
                        // Bound libavformat's own video packet queue during a microphone outage.
                        let mut options = av::Dictionary::new();
                        options.set("max_interleave_delta", "250000");
                        muxer.output.write_header_with(options)?;
                    } else {
                        muxer.output.write_header()?;
                    }
                    muxer.header_written = true;
                } else {
                    let mut encoded = av::Packet::copy(packet.data);
                    encoded.set_stream(0);
                    let time_base = muxer
                        .output
                        .stream(0)
                        .ok_or(Error::Invalid("missing video stream"))?
                        .time_base();
                    let pts = packet.timestamp_us.rescale_with(
                        (1, 1_000_000),
                        time_base,
                        av::Rounding::NearInfinity,
                    );
                    encoded.set_pts(Some(pts));
                    encoded.set_dts(Some(pts));
                    encoded.set_duration(50_000_i64.rescale((1, 1_000_000), time_base));
                    if packet.keyframe {
                        encoded.set_flags(av::packet::Flags::KEY);
                    }
                    if encoded.write_interleaved(&mut muxer.output).is_err() {
                        diagnostics::emit(
                            log_site!(),
                            Level::Warning,
                            format!(
                                "ts encoder write issue len: {} ts: {}",
                                packet.data.len(),
                                packet.timestamp_us
                            ),
                        );
                    }
                }
            }
        }
        Ok(())
    }

    pub fn write_audio(
        &mut self,
        data: &[u8],
        timestamp_us: u64,
        sample_rate: u32,
    ) -> Result<(), Error> {
        match &mut self.destination {
            Destination::Raw(_) => Ok(()),
            Destination::Mux(muxer) => {
                if let Some(audio) = &mut muxer.audio {
                    audio.push(data, timestamp_us, sample_rate)?;
                    if muxer.header_written {
                        audio.drain(&mut muxer.output)?;
                    }
                }
                Ok(())
            }
        }
    }

    pub fn close(self) -> Result<(), Error> {
        match self.destination {
            Destination::Raw(file) => {
                file.finish();
            }
            Destination::Mux(mut muxer) => {
                if !muxer.header_written {
                    return Err(Error::IncompleteVideo {
                        lock: self.lock.clone(),
                    });
                }
                if let Some(audio) = &mut muxer.audio {
                    audio.finish(&mut muxer.output)?;
                }
                if let Err(error) = muxer.output.write_trailer() {
                    diagnostics::emit(
                        log_site!(),
                        Level::Error,
                        format!("av_write_trailer failed {}", i32::from(error)),
                    );
                }
                if let Err(error) = codec::close_output(&mut muxer.output) {
                    diagnostics::emit(
                        log_site!(),
                        Level::Error,
                        format!("avio_closep failed {}", i32::from(error)),
                    );
                }
            }
        }
        fs::remove_file(self.lock)?;
        Ok(())
    }
}
