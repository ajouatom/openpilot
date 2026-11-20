use super::{protocol::receive, Client, Frame};
use crate::Error;
use std::{io::Write, os::unix::fs::FileExt, time::Duration};

impl Client {
    /// Update the N16 inputs before submitting the camera frame. An unchanged
    /// map stays in the shared input buffer until mapsd replaces or clears it.
    pub fn set_navigation(
        &mut self,
        navigation: &[f32; 47],
        map: Option<&[u8]>,
    ) -> Result<(), Error> {
        let address = |name| {
            self.info
                .layout
                .get(name)
                .map(|v| v.offset)
                .ok_or(Error::Contract("worker navigation input missing"))
        };
        let start = address("navigation")?;
        for (dst, value) in self.packed[start..start + 47 * 4]
            .chunks_exact_mut(4)
            .zip(navigation)
        {
            dst.copy_from_slice(&value.to_le_bytes());
        }
        if let Some(map) = map {
            if map.len() != 256 * 256 {
                return Err(Error::Contract("worker navigation map size"));
            }
            let start = address("nav_map")?;
            for (dst, pixel) in self.packed[start..start + map.len() * 4]
                .chunks_exact_mut(4)
                .zip(map)
            {
                dst.copy_from_slice(&f32::from(*pixel).to_le_bytes());
            }
        }
        Ok(())
    }

    pub(super) fn run_inner(
        &mut self,
        frame: Frame<'_>,
        prepare_only: bool,
    ) -> Result<bool, Error> {
        if frame.inputs.len() < 12
            || frame.main.len() != self.info.frame_size
            || frame.extra.len() != self.info.frame_size
        {
            return Err(Error::Contract("model worker input layout mismatch"));
        }
        for (target, value) in self.packed[..72]
            .chunks_exact_mut(4)
            .zip(frame.transforms.into_iter().flatten())
        {
            target.copy_from_slice(&value.to_le_bytes());
        }
        for (name, values) in [
            ("desire", &frame.inputs[..8]),
            ("traffic_convention", &frame.inputs[8..10]),
            ("action_t", &frame.inputs[10..12]),
        ] {
            let view = &self.info.layout[name];
            for (target, value) in self.packed[view.offset..view.offset + values.len() * 4]
                .chunks_exact_mut(4)
                .zip(values)
            {
                target.copy_from_slice(&value.to_le_bytes());
            }
        }
        for (name, bytes) in [("img", frame.main), ("big_img", frame.extra)] {
            let offset = self.info.layout[name].offset;
            self.packed[offset..offset + bytes.len()].copy_from_slice(bytes);
        }
        self.shared.as_file().write_all_at(&self.packed, 0)?;
        self.child
            .stdin
            .as_mut()
            .ok_or(Error::Contract("missing worker input"))?
            // carrot's worker knows only 'r': its fused graph advances history
            // and returns a full policy on catch-up frames too.
            .write_all(if prepare_only && !self.python {
                b"p"
            } else {
                b"r"
            })?;
        let stdout = self
            .child
            .stdout
            .as_mut()
            .ok_or(Error::Contract("missing worker output"))?;
        let response = receive(
            stdout,
            Duration::from_secs(if self.first { 20 } else { 1 }),
            &self.cancelled,
        )?;
        // '0': no new output this cycle (Rust worker: prepare only; carrot's
        // worker: it restored recurrent history and holds the last output).
        if response == b"0\n" && (prepare_only || (self.python && !self.first)) {
            return Ok(false);
        }
        if response != b"1\n" {
            return Err(Error::Contract("invalid model worker response"));
        }
        self.first = false;
        self.shared.as_file().read_exact_at(
            &mut self.output,
            u64::try_from(self.info.input_bytes)
                .map_err(|_| Error::Contract("worker input offset overflow"))?,
        )?;
        for (value, bytes) in self.values.iter_mut().zip(self.output.chunks_exact(4)) {
            *value = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        }
        if !self.values.iter().all(|value| value.is_finite()) {
            return Err(Error::Contract("non-finite model output"));
        }
        Ok(true)
    }
}
