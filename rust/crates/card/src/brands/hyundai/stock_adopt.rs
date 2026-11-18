//! carrot PR580 restart hand-over of a cruising camera stock SCC
//! (carstate `_update_stock_adopt`, carcontroller and cruise.py
//! `_adopt_stock_cruise`).
//!
//! The device (re)started while the camera's stock SCC was cruising (e.g. it
//! rebooted mid-drive). Keep stock driving and engage openpilot at the stock
//! set speed instead of cutting it off with ACCMode 0. While pending, card
//! sends no SCC_CONTROL (panda keeps forwarding the camera's own frame) and
//! the cruise helper requests an engage every 0.5 s at the stock speed,
//! subject to the normal interlocks and the cancel latch.
//!
//! The wait ends, and normal SCC_CONTROL resumes, when the driver or
//! openpilot asks for cruise off, when stock is not cruising as the camera
//! frame is first seen (a normal start), 30 s after start, or once openpilot
//! engages.

/// The restart hand-over window after the device starts (30 s at 100 Hz).
const WINDOW_FRAMES: u32 = 3000;

#[derive(Debug, Default)]
pub struct StockAdopt {
    pub pending: bool,
    /// Stock set speed (km/h) while waiting; 0 = nothing to adopt.
    pub kph: f64,
    frames: u32,
}

impl StockAdopt {
    /// Always on for camera-SCC CAN-FD with openpilot longitudinal.
    pub fn new(enabled: bool) -> Self {
        Self {
            pending: enabled,
            kph: 0.,
            frames: 0,
        }
    }

    /// Returns true while adopting: carstate then keeps MAIN on.
    pub fn update(
        &mut self,
        camera_seen: bool,
        stock_set_speed_ms: f64,
        off_requested: bool,
        main_mode: bool,
        acc_mode: f64,
    ) -> bool {
        self.kph = 0.;
        if !self.pending {
            return false;
        }
        if off_requested {
            self.pending = false;
            return false;
        }
        self.frames += 1;
        if self.frames > WINDOW_FRAMES {
            // Too late to be a restart hand-over: a normal start.
            self.pending = false;
            return false;
        }
        if !camera_seen {
            return false;
        }
        // 1 enabled, 2 driver override (gas).
        if !(main_mode && matches!(acc_mode as i32, 1 | 2)) {
            self.pending = false;
            return false;
        }
        self.kph = stock_set_speed_ms * 3.6;
        true
    }

    /// carcontroller: openpilot engaged, normal control continues.
    pub fn engaged(&mut self) {
        self.pending = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adopts_a_cruising_stock_scc_until_cancelled() {
        let mut adopt = StockAdopt::new(true);
        assert!(
            !adopt.update(false, 25., false, false, 0.),
            "camera not seen yet"
        );
        assert!(adopt.pending);
        assert!(adopt.update(true, 25., false, true, 1.));
        assert!((adopt.kph - 90.).abs() < 1e-9);
        assert!(!adopt.update(true, 25., true, true, 1.), "driver cancel");
        assert!(!adopt.pending);
    }

    #[test]
    fn a_normal_start_never_adopts() {
        let mut adopt = StockAdopt::new(true);
        assert!(!adopt.update(true, 0., false, false, 0.));
        assert!(!adopt.pending);
        let mut late = StockAdopt::new(true);
        for _ in 0..WINDOW_FRAMES {
            late.update(false, 0., false, false, 0.);
        }
        assert!(
            !late.update(true, 25., false, true, 1.),
            "after the 30 s window"
        );
    }
}
