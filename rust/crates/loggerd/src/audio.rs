use crate::diagnostics;
use crate::Error;
use ffmpeg_next::{self as av, format::context::Output};
use openpilot_logging::{log_site, record::Level};

pub struct Audio {
    encoder: av::codec::encoder::audio::Encoder,
    buffer: Vec<f32>,
    pts: i64,
    stream: usize,
}

impl Audio {
    pub fn new(output: &mut Output, sample_rate: u32) -> Result<Self, Error> {
        if sample_rate == 0 {
            return Err(Error::Invalid("zero audio sample rate"));
        }
        let codec = av::encoder::find(av::codec::Id::AAC).ok_or(av::Error::EncoderNotFound)?;
        let mut encoder = av::codec::Context::new_with_codec(codec)
            .encoder()
            .audio()?;
        encoder.set_format(av::format::Sample::F32(av::format::sample::Type::Planar));
        encoder.set_rate(i32::try_from(sample_rate)?);
        encoder.set_channel_layout(av::ChannelLayout::MONO);
        encoder.set_bit_rate(32_000);
        encoder.set_flags(av::codec::Flags::GLOBAL_HEADER);
        encoder.set_time_base((1, i32::try_from(sample_rate)?));
        // FFmpeg's AAC encoder defaults to the "twoloop" coder, which spent
        // 23 % of loggerd's CPU on a comma 3X (simpleperf) for 32 kbit/s mono
        // cabin audio; "fast" codes it in a fraction of that.
        let mut options = av::Dictionary::new();
        options.set("aac_coder", "fast");
        let encoder = encoder.open_as_with(codec, options)?;
        av::log::set_level(av::log::Level::Warning);
        let mut stream = output.add_stream(codec)?;
        stream.set_parameters(&encoder);
        let stream = stream.index();
        Ok(Self {
            encoder,
            buffer: Vec::new(),
            pts: 0,
            stream,
        })
    }

    pub fn push(&mut self, bytes: &[u8], timestamp_us: u64, sample_rate: u32) -> Result<(), Error> {
        // A different format needs a new segment header. The next writer
        // opens at the observed rate; never encode PCM at the wrong speed.
        if self.encoder.rate() != sample_rate {
            return Ok(());
        }
        if self.pts == 0 {
            self.pts = i64::try_from(
                timestamp_us
                    .checked_mul(u64::from(self.encoder.rate()))
                    .ok_or(Error::Invalid("audio timestamp overflow"))?
                    / 1_000_000,
            )?;
        }
        let sample_count = bytes.len() / 2;
        let maximum = usize::try_from(sample_rate)?
            .checked_mul(10)
            .ok_or(Error::Invalid("audio queue overflow"))?;
        let combined = self
            .buffer
            .len()
            .checked_add(sample_count)
            .ok_or(Error::Invalid("audio queue overflow"))?;
        let discard = combined.saturating_sub(maximum);
        if discard > self.buffer.len() {
            return Err(Error::Invalid("audio packet exceeds ten seconds"));
        }
        if discard > 0 {
            self.buffer.drain(..discard);
            self.pts = self
                .pts
                .checked_add(i64::try_from(discard)?)
                .ok_or(Error::Invalid("audio timestamp overflow"))?;
            diagnostics::emit(
                log_site!(),
                Level::Error,
                format!("Audio buffer overflow, dropping {discard} oldest samples"),
            );
        }
        self.buffer.extend(
            bytes.chunks_exact(2).map(|sample| {
                f32::from(i16::from_le_bytes([sample[0], sample[1]])) * (1.0 / 32768.0)
            }),
        );
        Ok(())
    }

    pub fn drain(&mut self, output: &mut Output) -> Result<(), Error> {
        let count = usize::try_from(self.encoder.frame_size())?;
        if count == 0 {
            return Err(Error::Invalid("AAC encoder returned zero frame size"));
        }
        while self.buffer.len() >= count {
            self.encode_frame(output, count)?;
        }
        Ok(())
    }

    fn encode_frame(&mut self, output: &mut Output, count: usize) -> Result<(), Error> {
        let mut frame =
            av::frame::Audio::new(self.encoder.format(), count, av::ChannelLayout::MONO);
        frame.set_rate(self.encoder.rate());
        frame.set_pts(Some(self.pts));
        frame.plane_mut::<f32>(0)[..count].copy_from_slice(&self.buffer[..count]);
        self.buffer.drain(..count);
        if let Err(error) = self.encoder.send_frame(&frame) {
            diagnostics::emit(
                log_site!(),
                Level::Warning,
                format!(
                    "AUDIO: Failed to send audio frame to encoder: {}",
                    i32::from(error)
                ),
            );
        } else {
            self.receive(output);
        }
        self.pts = self
            .pts
            .checked_add(i64::try_from(count)?)
            .ok_or(Error::Invalid("audio timestamp overflow"))?;
        Ok(())
    }

    fn receive(&mut self, output: &mut Output) {
        let mut packet = av::Packet::empty();
        while self.encoder.receive_packet(&mut packet).is_ok() {
            if let Some(stream) = output.stream(self.stream) {
                packet.rescale_ts(self.encoder.time_base(), stream.time_base());
                packet.set_stream(self.stream);
                if let Err(error) = packet.write_interleaved(output) {
                    diagnostics::emit(
                        log_site!(),
                        Level::Warning,
                        format!("AUDIO: Write frame failed - error: {}", i32::from(error)),
                    );
                }
            }
        }
    }

    pub fn finish(&mut self, output: &mut Output) -> Result<(), Error> {
        let frame_size = usize::try_from(self.encoder.frame_size())?;
        if !self.buffer.is_empty() && self.buffer.len() < frame_size {
            self.buffer.resize(frame_size, 0.0);
            self.encode_frame(output, frame_size)?;
        }
        if let Err(error) = self.encoder.send_eof() {
            diagnostics::emit(
                log_site!(),
                Level::Warning,
                format!(
                    "AUDIO: Failed to send audio frame to encoder: {}",
                    i32::from(error)
                ),
            );
        } else {
            self.receive(output);
        }
        Ok(())
    }
}
