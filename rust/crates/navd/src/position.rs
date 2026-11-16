//! carrot's `selfdrive/navigation/helpers.py` position fusion, shared by navd
//! and mapsd: the phone/navigation position from `carrotMan` while it is
//! fresh, else the freshest usable GPS fix, and a parked anchor that keeps
//! the last fix while the car stays in P (never an old fix at startup, never
//! dead reckoning while moving).

/// Web Mercator's latitude limit.
pub const MAX_LATITUDE: f64 = 85.05112878;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigationPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub bearing: f64,
    /// m/s, never negative.
    pub speed: f64,
    /// When the position held (monotonic seconds).
    pub time: f64,
    /// When its fix was taken (monotonic seconds).
    pub fix_time: f64,
}

/// `carrotMan` as the fusion reads it.
#[derive(Clone, Debug, Default)]
pub struct CarrotFix {
    pub valid: bool,
    /// Monotonic seconds the message arrived.
    pub received: f64,
    pub position_source_empty: bool,
    pub heading_valid: bool,
    pub fix_mono_ns: u64,
    pub position_mono_ns: u64,
    pub latitude: f64,
    pub longitude: f64,
    pub angle: f64,
    /// km/h.
    pub speed: f64,
}

/// `gpsLocation` / `gpsLocationExternal`.
#[derive(Clone, Debug, Default)]
pub struct GpsFix {
    pub valid: bool,
    pub has_fix: bool,
    pub received: f64,
    pub log_mono_time: u64,
    pub latitude: f64,
    pub longitude: f64,
    pub bearing: f64,
    pub speed: f64,
}

/// `carState` as the parked anchor reads it.
#[derive(Clone, Debug, Default)]
pub struct CarPark {
    pub valid: bool,
    pub alive: bool,
    pub received: f64,
    pub park: bool,
    pub standstill: bool,
    pub v_ego: f64,
}

/// One update's inputs: what each service last said, and now.
#[derive(Clone, Debug, Default)]
pub struct Sample {
    pub now: f64,
    pub carrot: Option<CarrotFix>,
    pub gps: Vec<GpsFix>,
    pub car: Option<CarPark>,
}

fn in_bounds(latitude: f64, longitude: f64) -> bool {
    latitude.abs() <= MAX_LATITUDE && longitude.abs() <= 180.
}

fn fresh(now: f64, at: f64, limit: f64) -> bool {
    (0. ..limit).contains(&(now - at))
}

/// The freshest usable GPS fix, as `latest_gps_source`.
pub fn latest_gps(sample: &Sample) -> Option<&GpsFix> {
    sample
        .gps
        .iter()
        .filter(|gps| {
            gps.valid
                && gps.has_fix
                && fresh(sample.now, gps.received, 2.)
                && [gps.latitude, gps.longitude, gps.bearing, gps.speed]
                    .iter()
                    .all(|value| value.is_finite())
                && in_bounds(gps.latitude, gps.longitude)
        })
        .max_by_key(|gps| gps.log_mono_time)
}

/// `navigation_position`: carrot's phone/navigation fusion bounded by the
/// original fix's freshness, else the freshest GPS.
///
/// Deviation (trial): carrot-nav drops a carrot position without a heading.
/// The stock-navigation (vehicle CAN) position has none while the car stands
/// still, so after a cold start in a car park neither navd nor mapsd had a
/// position until the car moved or the comma's GPS got a fix. A heading-less
/// position is now taken while fresh carState says the car stands still; its
/// bearing is NaN, so navd asks Mapbox without a start bearing.
pub fn navigation_position(sample: &Sample) -> Option<NavigationPosition> {
    let now = sample.now;
    if let Some(carrot) = &sample.carrot {
        let headed = carrot.position_source_empty || carrot.heading_valid;
        if carrot.valid
            && fresh(now, carrot.received, 0.5)
            && (headed || standing_still(sample))
            && carrot.fix_mono_ns > 0
            && fresh(now, carrot.fix_mono_ns as f64 / 1e9, 2.)
            && fresh(now, carrot.position_mono_ns as f64 / 1e9, 0.5)
            && [
                carrot.latitude,
                carrot.longitude,
                carrot.angle,
                carrot.speed,
            ]
            .iter()
            .all(|value| value.is_finite())
            && in_bounds(carrot.latitude, carrot.longitude)
        {
            return Some(NavigationPosition {
                latitude: carrot.latitude,
                longitude: carrot.longitude,
                bearing: if headed { carrot.angle } else { f64::NAN },
                speed: (carrot.speed / 3.6).max(0.),
                time: carrot.position_mono_ns as f64 / 1e9,
                fix_time: carrot.fix_mono_ns as f64 / 1e9,
            });
        }
    }
    let gps = latest_gps(sample)?;
    Some(NavigationPosition {
        latitude: gps.latitude,
        longitude: gps.longitude,
        bearing: gps.bearing,
        speed: gps.speed.max(0.),
        time: gps.received,
        fix_time: gps.received,
    })
}

/// Fresh, valid carState with the car standing still.
fn standing_still(sample: &Sample) -> bool {
    sample.car.as_ref().is_some_and(|car| {
        car.valid
            && car.alive
            && fresh(sample.now, car.received, 0.5)
            && car.standstill
            && car.v_ego.is_finite()
            && car.v_ego.abs() < 0.1
    })
}

/// `ParkedPosition`: keeps an observed parked position. Fresh carState must
/// keep confirming P and zero speed; leaving P or losing that evidence clears
/// the anchor before it can be reused.
#[derive(Clone, Debug, Default)]
pub struct ParkedPosition {
    anchor: Option<NavigationPosition>,
    parked_since: Option<f64>,
    checked: Option<f64>,
    /// The last update returned the anchor rather than a live position.
    pub held: bool,
}

impl ParkedPosition {
    pub fn update(&mut self, sample: &Sample, max_poll_gap: f64) -> Option<NavigationPosition> {
        let now = sample.now;
        let parked = sample.car.as_ref().is_some_and(|car| {
            car.valid
                && car.alive
                && fresh(now, car.received, 0.5)
                && car.park
                && car.standstill
                && car.v_ego.is_finite()
                && car.v_ego.abs() < 0.01
        });
        let continuous = self
            .checked
            .is_some_and(|checked| (0. ..=max_poll_gap).contains(&(now - checked)));
        if !parked || !continuous {
            self.anchor = None;
            self.parked_since = parked.then_some(now);
        }
        self.checked = Some(now);
        let position = navigation_position(sample);
        self.held = false;
        if let Some(position) = position {
            self.anchor = match self.parked_since {
                Some(since)
                    if parked
                        && now - since >= 1.
                        && position.fix_time >= since
                        && position.speed < 0.1 =>
                {
                    Some(NavigationPosition {
                        speed: 0.,
                        ..position
                    })
                }
                _ => None,
            };
            return Some(position);
        }
        self.held = parked && self.anchor.is_some();
        if self.held {
            self.anchor
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gps(received: f64, log: u64, latitude: f64) -> GpsFix {
        GpsFix {
            valid: true,
            has_fix: true,
            received,
            log_mono_time: log,
            latitude,
            longitude: 127.,
            bearing: 90.,
            speed: 0.,
        }
    }

    fn parked(now: f64) -> CarPark {
        CarPark {
            valid: true,
            alive: true,
            received: now,
            park: true,
            standstill: true,
            v_ego: 0.,
        }
    }

    #[test]
    fn freshest_gps_wins_and_stale_or_bad_fixes_are_ignored() {
        let mut sample = Sample {
            now: 10.,
            gps: vec![gps(9.5, 5, 37.), gps(9.9, 9, 36.)],
            ..Sample::default()
        };
        assert_eq!(navigation_position(&sample).unwrap().latitude, 36.);
        sample.gps[1].received = 7.; // older than 2 s
        assert_eq!(navigation_position(&sample).unwrap().latitude, 37.);
        sample.gps[0].latitude = 90.; // out of bounds
        assert!(navigation_position(&sample).is_none());
    }

    #[test]
    fn carrot_position_needs_a_fresh_fix_and_heading() {
        let mut carrot = CarrotFix {
            valid: true,
            received: 9.9,
            position_source_empty: false,
            heading_valid: true,
            fix_mono_ns: 9_000_000_000,
            position_mono_ns: 9_800_000_000,
            latitude: 35.,
            longitude: 128.,
            angle: 10.,
            speed: 36.,
        };
        let sample = Sample {
            now: 10.,
            carrot: Some(carrot.clone()),
            gps: vec![gps(9.9, 1, 37.)],
            ..Sample::default()
        };
        let position = navigation_position(&sample).unwrap();
        assert_eq!((position.latitude, position.speed), (35., 10.));
        carrot.heading_valid = false;
        let sample = Sample {
            carrot: Some(carrot),
            ..sample
        };
        assert_eq!(
            navigation_position(&sample).unwrap().latitude,
            37.,
            "falls back to GPS"
        );
    }

    #[test]
    fn headingless_carrot_position_is_taken_only_while_standing_still() {
        let carrot = CarrotFix {
            valid: true,
            received: 9.9,
            position_source_empty: false,
            heading_valid: false,
            fix_mono_ns: 9_000_000_000,
            position_mono_ns: 9_800_000_000,
            latitude: 35.,
            longitude: 128.,
            angle: 270.,
            speed: 0.,
        };
        // Cold start in a car park: no GPS fix, the vehicle position has no heading.
        let mut sample = Sample {
            now: 10.,
            carrot: Some(carrot),
            car: Some(parked(10.)),
            ..Sample::default()
        };
        let position = navigation_position(&sample).unwrap();
        assert_eq!(position.latitude, 35.);
        assert!(position.bearing.is_nan(), "no stale start bearing");
        // Moving (or no fresh carState): the original rule, no position.
        sample.car = Some(CarPark {
            standstill: false,
            v_ego: 3.,
            ..parked(10.)
        });
        assert!(navigation_position(&sample).is_none());
        sample.car = Some(parked(9.));
        assert!(navigation_position(&sample).is_none());
    }

    #[test]
    fn parked_anchor_holds_a_fix_seen_while_parked_and_clears_on_leaving_p() {
        let mut held = ParkedPosition::default();
        let sample = |now: f64, fix: bool, car: CarPark| Sample {
            now,
            gps: if fix {
                vec![gps(now, now as u64, 37.)]
            } else {
                Vec::new()
            },
            car: Some(car),
            ..Sample::default()
        };
        assert!(held.update(&sample(0., true, parked(0.)), 0.5).is_some());
        assert!(held.update(&sample(0.4, true, parked(0.4)), 0.5).is_some());
        assert!(held.update(&sample(0.8, true, parked(0.8)), 0.5).is_some());
        // a fix seen a full second into the park is anchored
        assert!(held.update(&sample(1.2, true, parked(1.2)), 0.5).is_some());
        let kept = held.update(&sample(1.6, false, parked(1.6)), 0.5);
        assert!(held.held);
        assert_eq!(kept.unwrap().latitude, 37.);
        let mut moving = parked(2.0);
        moving.park = false;
        assert!(held.update(&sample(2.0, false, moving), 0.5).is_none());
        assert!(!held.held);
    }

    #[test]
    fn no_anchor_from_a_fix_older_than_the_park() {
        let mut held = ParkedPosition::default();
        let stale = |now: f64| Sample {
            now,
            gps: vec![gps(0.0, 1, 37.)],
            car: Some(parked(now)),
            ..Sample::default()
        };
        // P from 0.5 s; the only fix is from 0.0 s, before the park.
        for now in [0.5, 0.9, 1.3, 1.7] {
            held.update(&stale(now), 0.5);
        }
        let none = Sample {
            now: 2.4,
            car: Some(parked(2.4)),
            ..Sample::default()
        };
        assert!(held.update(&none, 1.).is_none());
    }
}
