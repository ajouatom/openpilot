//! carrot `CameraSetSpeedSync` / `CameraGapSync` / `CameraSync` (hyundaicanfd.py,
//! PR580 00d9cae3b): keep the camera's stock SCC set speed and gap equal to
//! the driver's cruise set speed and openpilot's gap on a camera-SCC CAN-FD car
//! with openpilot longitudinal, so a device drop-out hands the car the same
//! speed and gap. Presses go to the camera bus only and never change our own
//! SCC output or set speed. See carrot's docstrings for each rule's failure.

/// Stock SCC set range (km/h or mph units); a target outside is left alone.
const SPEED_MIN: i64 = 30;
const SPEED_MAX: i64 = 180;
/// Two full gap cycles; beyond that the camera is not following.
const GAP_MAX_PRESSES: u32 = 8;
/// Syncing resumes only 1 s after any driver button press.
const DRIVER_HOLD_FRAMES: i64 = 100;
/// A different sync button starts only 0.3 s after the last sync press.
const SWITCH_FRAMES: i64 = 30;

pub const RES_ACCEL: u8 = 1;
pub const SET_DECEL: u8 = 2;
pub const GAP_DIST: u8 = 3;

#[derive(Default)]
struct SetSpeed {
    target: Option<i64>,
    direction: i8,
    done: bool,
    /// Still working toward the target: the gap sync waits.
    busy: bool,
}

impl SetSpeed {
    /// All speeds in the cluster unit. 1 (+), 2 (-) or 0.
    fn button(&mut self, target_speed: f64, camera_speed: f64, frame: u32, v_ego: f64) -> u8 {
        let target = target_speed.round() as i64;
        if self.target != Some(target) {
            self.target = Some(target);
            self.direction = 0;
            self.done = false;
        }
        let camera = camera_speed as i64;
        if camera == target {
            // Reached: a later mismatch (driver press) is a new approach.
            self.direction = 0;
        }
        self.busy = !self.done
            && (SPEED_MIN..=SPEED_MAX).contains(&target)
            && 0 < camera
            && camera < 255
            && camera != target;
        if !self.busy {
            return 0;
        }
        let direction: i8 = if target > camera { 1 } else { -1 };
        if self.direction != 0 && direction != self.direction {
            // Overshot: this car's short press is not a single unit.
            self.done = true;
            self.busy = false;
            return 0;
        }
        // Jump guard: while faster than VSetDis a stock press sets VSetDis to
        // the current speed instead of stepping it.
        if v_ego.round() as i64 > if direction > 0 { target } else { camera } {
            return 0;
        }
        self.direction = direction;
        if !(10 < frame % 30 && frame % 30 <= 16) {
            return 0;
        }
        if direction > 0 {
            RES_ACCEL
        } else {
            SET_DECEL
        }
    }
}

#[derive(Default)]
struct Gap {
    target: Option<i64>,
    presses: u32,
    window: Option<u32>,
}

impl Gap {
    fn button(&mut self, target_bars: f64, camera_gap: f64, frame: u32) -> u8 {
        let (target, camera) = (target_bars as i64, camera_gap as i64);
        if self.target != Some(target) || camera == target {
            self.target = Some(target);
            self.presses = 0;
        }
        if camera == target
            || !((1..=4).contains(&target) && (1..=4).contains(&camera))
            || self.presses >= GAP_MAX_PRESSES
        {
            return 0;
        }
        if !(10 < frame % 100 && frame % 100 <= 16) {
            return 0;
        }
        if self.window != Some(frame / 100) {
            self.window = Some(frame / 100);
            self.presses += 1;
        }
        GAP_DIST
    }
}

/// What one frame's sync decision reads.
pub struct SyncInput {
    /// SCC_CONTROL VSetDis and DISTANCE_SETTING.
    pub camera_set: f64,
    pub camera_gap: f64,
    pub acc_mode: f64,
    /// CarState.vCruiseCluster (km/h).
    pub set_kph: f64,
    pub v_ego: f64,
    pub v_ego_cluster: f64,
    pub metric: bool,
    /// hudControl.leadDistanceBars.
    pub bars: f64,
}

pub struct CameraSync {
    speed: SetSpeed,
    gap: Gap,
    driver_frame: i64,
    last_button: u8,
    last_frame: i64,
}

impl Default for CameraSync {
    fn default() -> Self {
        Self {
            speed: SetSpeed::default(),
            gap: Gap::default(),
            driver_frame: -DRIVER_HOLD_FRAMES - 1,
            last_button: 0,
            last_frame: -SWITCH_FRAMES - 1,
        }
    }
}

impl CameraSync {
    pub fn note_driver(&mut self, frame: u32, pressed: bool) {
        if pressed {
            self.driver_frame = i64::from(frame);
        }
    }

    /// The camera-bus button for this frame, 0 for none. Every gate only
    /// removes presses.
    pub fn button(&mut self, i: &SyncInput, frame: u32) -> u8 {
        let now = i64::from(frame);
        if now - self.driver_frame <= DRIVER_HOLD_FRAMES
            || i.acc_mode != 1.
            || !(0. < i.camera_set && i.camera_set < 255.)
            || !(0. < i.set_kph && i.set_kph < 255.)
            || i.v_ego <= 3.
        {
            return 0;
        }
        let recent = now - self.last_frame <= SWITCH_FRAMES;
        if !(recent && self.last_button == GAP_DIST) {
            let unit = if i.metric {
                1.
            } else {
                1. / 1.609_344
            };
            let button = self.speed.button(
                i.set_kph * unit,
                i.camera_set,
                frame,
                i.v_ego_cluster * 3.6 * unit,
            );
            if button != 0 || self.speed.busy {
                return self.sent(button, now);
            }
        }
        if recent && matches!(self.last_button, RES_ACCEL | SET_DECEL) {
            return 0;
        }
        let button = self.gap.button(i.bars, i.camera_gap, frame);
        self.sent(button, now)
    }

    fn sent(&mut self, button: u8, now: i64) -> u8 {
        if button != 0 {
            self.last_button = button;
            self.last_frame = now;
        }
        button
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn press_frame(base: u32) -> u32 {
        base * 30 + 12
    }

    #[test]
    fn set_speed_steps_toward_the_target_with_a_jump_guard_and_stops_on_overshoot() {
        let mut s = SetSpeed::default();
        assert_eq!(s.button(80., 70., press_frame(1), 60.), RES_ACCEL);
        // Outside the 0.3 s press window: no press, still busy.
        assert_eq!(s.button(80., 70., 30, 60.), 0);
        assert!(s.busy);
        // Faster than the target: a + press would set VSetDis to the speed.
        assert_eq!(s.button(80., 70., press_frame(2), 85.), 0);
        assert_eq!(s.button(80., 85., press_frame(3), 60.), 0, "overshoot");
        assert!(s.done && !s.busy);
        assert_eq!(
            s.button(80., 90., press_frame(4), 60.),
            0,
            "done for this target"
        );
        assert_eq!(
            s.button(70., 90., press_frame(5), 60.),
            SET_DECEL,
            "a new target clears it"
        );
        assert_eq!(s.button(200., 90., press_frame(6), 60.), 0, "out of range");
        assert_eq!(s.button(90., 255., press_frame(7), 60.), 0, "MAIN off");
    }

    #[test]
    fn gap_presses_once_a_second_and_gives_up_after_two_cycles() {
        let mut g = Gap::default();
        let mut presses = 0;
        for second in 0..20 {
            if g.button(2., 3., second * 100 + 12) == GAP_DIST {
                presses += 1;
            }
        }
        assert_eq!(presses, GAP_MAX_PRESSES);
        assert_eq!(g.button(2., 2., 2012), 0);
        assert_eq!(
            g.button(2., 3., 2112),
            GAP_DIST,
            "matched once: counting restarts"
        );
    }

    fn input() -> SyncInput {
        SyncInput {
            camera_set: 70.,
            camera_gap: 2.,
            acc_mode: 1.,
            set_kph: 80.,
            v_ego: 15.,
            v_ego_cluster: 15.,
            metric: true,
            bars: 3.,
        }
    }

    #[test]
    fn sync_waits_for_the_driver_then_does_speed_before_gap() {
        let mut sync = CameraSync::default();
        sync.note_driver(1000, true);
        assert_eq!(sync.button(&input(), 1042), 0, "driver pressed within 1 s");
        assert_eq!(
            sync.button(&input(), 1102),
            0,
            "busy outside the press window"
        );
        assert_eq!(sync.button(&input(), 1121), RES_ACCEL);
        let matched = SyncInput {
            camera_set: 80.,
            ..input()
        };
        assert_eq!(sync.button(&matched, 1140), 0, "0.3 s after a speed press");
        assert_eq!(sync.button(&matched, 1212), GAP_DIST);
        let parked = SyncInput {
            v_ego: 1.,
            ..input()
        };
        assert_eq!(sync.button(&parked, 1312), 0, "standstill: + would resume");
        let override_ = SyncInput {
            acc_mode: 2.,
            ..input()
        };
        assert_eq!(sync.button(&override_, 1342), 0, "gas override");
    }
}
