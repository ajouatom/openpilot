/// An encoder stream counts toward a rotation while its packets keep coming.
const STREAM_ALIVE_NS: u64 = 2_000_000_000;

#[derive(Default)]
pub struct Rotation {
    pub ready: usize,
    pub last_rotation_ns: u64,
    pub last_camera_ns: u64,
    pub test_mode: bool,
    /// Last packet time of each encoder stream (road, wide, driver, qcamera).
    pub stream_seen_ns: [u64; 4],
}

impl Rotation {
    pub fn timeout_reason(&self, now_ns: u64) -> Option<&'static str> {
        let duration = now_ns.saturating_sub(self.last_rotation_ns);
        if !self.test_mode && duration > 60_000_000_000 {
            if now_ns.saturating_sub(self.last_camera_ns) > 500_000_000 {
                return Some("no camera packets seen. auto rotating");
            }
            if duration > 72_000_000_000 {
                return Some("segment too long. auto rotating");
            }
        }
        None
    }

    /// Rotates once every encoder that is sending packets is ready. A camera
    /// that never comes up (a C3X Lite or a failed driver camera sensor) is
    /// not waited for: with four required, every segment ran to the 72 s
    /// timeout while encoderd moved on every 60 s, the encoders ran further
    /// ahead each segment and from the fifth segment on no video was written
    /// at all (KIA K5 route 0000027e: no EncodeIdx in any rlog).
    pub fn due(&self, now_ns: u64) -> bool {
        let alive = self
            .stream_seen_ns
            .iter()
            .filter(|&&seen| seen != 0 && now_ns.saturating_sub(seen) < STREAM_ALIVE_NS)
            .count();
        let expected = if alive == 0 { 4 } else { alive };
        self.ready >= expected || self.timeout_reason(now_ns).is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotates_when_all_four_encoders_are_ready() {
        let state = Rotation {
            ready: 4,
            ..Rotation::default()
        };
        let due = state.due(0);
        assert!(due);
    }

    #[test]
    fn a_camera_without_packets_is_not_waited_for() {
        let now = 30_000_000_000;
        let mut state = Rotation {
            ready: 3,
            stream_seen_ns: [now - 10_000_000, now - 20_000_000, 0, now - 30_000_000],
            ..Rotation::default()
        };
        assert!(state.due(now));
        // A driver camera that stopped long ago does not count either.
        state.stream_seen_ns[2] = now - 5_000_000_000;
        assert!(state.due(now));
        // All four sending: the fourth is still waited for.
        state.stream_seen_ns[2] = now - 10_000_000;
        assert!(!state.due(now));
        state.ready = 4;
        assert!(state.due(now));
    }

    #[test]
    fn rotates_only_beyond_source_timeout_boundaries() {
        for (now_ns, camera_ns, expected) in [
            (60_000_000_000, 0, false),
            (60_000_000_001, 0, true),
            (61_000_000_000, 60_500_000_000, false),
            (61_000_000_001, 60_500_000_000, true),
            (72_000_000_000, 72_000_000_000, false),
            (72_000_000_001, 72_000_000_001, true),
        ] {
            let state = Rotation {
                last_camera_ns: camera_ns,
                ..Rotation::default()
            };
            let due = state.due(now_ns);
            assert_eq!(due, expected, "now={now_ns}, camera={camera_ns}");
        }
    }

    #[test]
    fn waits_for_encoders_when_source_test_mode_disables_timeout() {
        let state = Rotation {
            test_mode: true,
            ..Rotation::default()
        };
        let due = state.due(1_000_000_000_000);
        assert!(!due);
    }
}
