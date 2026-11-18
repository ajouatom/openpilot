mod events;
mod limits;
pub mod policy;
mod profile;
mod pv5;
pub mod section_average;
pub mod speedcam;
use super::{parameters::setting_int, wire::Values, Error};
use openpilot_params::Params;
pub use profile::{EventType, Profile};

#[derive(Clone, Debug)]
pub struct Event {
    pub id: u64,
    pub kind: EventType,
    pub speed: f64,
    pub subtype: u8,
    pub target: f64,
    pub cancelled: bool,
}

#[derive(Default, Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Output {
    pub speed_limit: f64,
    pub speed_limit_distance: f64,
    pub bump_distance: f64,
    pub school_zone: bool,
    pub active: bool,
    pub section_active: bool,
    pub available: bool,
    pub speed: f64,
}

pub struct Input<'a> {
    pub hda: Option<&'a Values>,
    pub position: Option<&'a Values>,
    pub segment: Option<&'a Values>,
    pub profile: Option<&'a Values>,
    pub status: Option<&'a Values>,
    pub profile_timestamp: u64,
    pub position_timestamp: u64,
    pub segment_timestamp: u64,
    pub hda_timestamp: u64,
    pub status_timestamp: u64,
    pub last_update: u64,
    pub pt_timeout: bool,
    pub alt_timeout: bool,
    pub hda_size: usize,
    pub status_size: usize,
    pub metric: bool,
}

pub struct Navigation {
    pub camera_policy: speedcam::SpeedcamPolicy,
    /// KIA_PV5: wrapped 0x364/0x093 with the 0x380 section status.
    pub wrapped: bool,
    /// KIA_EV4 (carrot 693efb885/e48fff204): wrapped 0x364/0x099/0x093;
    /// position stays on 0x4B4.
    pub ev4: bool,
    pub total_distance: f64,
    pub speed_limit_distance: f64,
    pub distance_time: f64,
    pub mode: i32,
    pub school_control: bool,
    pub params_counter: u32,
    pub events: Vec<Event>,
    pub segment_timestamp: u64,
    pub profile_timestamp: u64,
    pub available: bool,
    pub route_reset_timestamp: u64,
    pub route_state: u8,
    pub route_path: Option<u8>,
    pub road_class: u8,
    pub camera_target: Option<f64>,
    pub camera_event: Option<Event>,
    pub camera_speed: f64,
    pub camera_status_target: Option<f64>,
    pub speed_zone_active: bool,
    pub speed_zone_speed: f64,
    pub school_active: bool,
    pub school_start_distance: f64,
    pub school_uses_status: bool,
    pub pv5_section_previous: bool,
    pub pv5_completed_speed: f64,
    next_event: u64,
}

impl Navigation {
    pub fn new(wrapped: bool, ev4: bool, settings: &Params) -> Result<Self, Error> {
        Ok(Self::with(
            wrapped,
            ev4,
            f64::from(setting_int(settings, "VehicleSpeedCameraDistanceTime")?.clamp(10, 200))
                / 10.,
            setting_int(settings, "VehicleNaviCanControl")?.clamp(0, 3),
            settings.get_bool("VehicleNaviSchoolZoneControl")?,
        ))
    }

    fn with(wrapped: bool, ev4: bool, distance_time: f64, mode: i32, school_control: bool) -> Self {
        Self {
            camera_policy: speedcam::SpeedcamPolicy::default(),
            wrapped,
            ev4,
            total_distance: 0.,
            speed_limit_distance: 0.,
            distance_time,
            mode,
            school_control,
            params_counter: 0,
            events: Vec::with_capacity(32),
            segment_timestamp: 0,
            profile_timestamp: 0,
            available: false,
            route_reset_timestamp: 0,
            route_state: 0,
            route_path: None,
            road_class: 7,
            camera_target: None,
            camera_event: None,
            camera_speed: 0.,
            camera_status_target: None,
            speed_zone_active: false,
            speed_zone_speed: 0.,
            school_active: false,
            school_start_distance: 0.,
            school_uses_status: false,
            pv5_section_previous: false,
            pv5_completed_speed: 0.,
            next_event: 0,
        }
    }

    pub fn controlled_access(&self, input: &Input<'_>) -> bool {
        let link = input
            .hda
            .and_then(|data| data.get("LinkClass"))
            .copied()
            .unwrap_or(0.);
        [1., 2., 3.].contains(&link) || [1, 2].contains(&self.road_class)
    }

    pub fn clear_events(&mut self) {
        self.camera_policy.clear();
        self.events.clear();
        self.camera_target = None;
        self.camera_event = None;
    }
    pub fn clear_school(&mut self) {
        self.school_active = false;
        self.school_start_distance = self.total_distance;
        self.school_uses_status = false;
    }
    pub fn clear_speed_zone(&mut self) {
        self.speed_zone_active = false;
        self.speed_zone_speed = 0.;
    }
    pub fn clear_route_filtered(&mut self) {
        if self.mode < 2 {
            return;
        }
        self.events.retain(|event| match event.kind {
            EventType::Camera => self.mode == 2,
            EventType::Bump => false,
            EventType::SpeedZone => true,
        });
        if self
            .camera_event
            .as_ref()
            .is_some_and(|cached| !self.events.iter().any(|event| event.id == cached.id))
        {
            self.camera_event = None;
            self.camera_target = None;
        }
        if self.mode == 3 {
            self.clear_speed_zone();
        }
    }

    pub fn refresh(&mut self, settings: &Params) -> Result<bool, Error> {
        self.params_counter += 1;
        if self.params_counter < 100 {
            return Ok(false);
        }
        self.params_counter = 0;
        let distance_time =
            f64::from(setting_int(settings, "VehicleSpeedCameraDistanceTime")?.clamp(10, 200))
                / 10.;
        let changed = distance_time != self.distance_time;
        self.distance_time = distance_time;
        if changed && self.camera_status_target.is_some() {
            self.camera_status_target =
                Some(self.total_distance + self.camera_speed * distance_time);
        }
        let mode = setting_int(settings, "VehicleNaviCanControl")?.clamp(0, 3);
        if mode != self.mode {
            self.mode = mode;
            if mode == 0 {
                self.clear_events();
                self.clear_speed_zone();
            } else if mode >= 2 {
                self.clear_route_filtered();
            }
        }
        let school = settings.get_bool("VehicleNaviSchoolZoneControl")?;
        if school != self.school_control {
            self.school_control = school;
            if !school {
                self.clear_school();
            }
        }
        Ok(changed)
    }
}

#[cfg(test)]
mod ev4_tests {
    use super::{Input, Navigation, Output, Values};

    fn input<'a>(segment: Option<&'a Values>, segment_ts: u64, hda_ts: u64) -> Input<'a> {
        Input {
            hda: None,
            position: None,
            segment,
            profile: None,
            status: None,
            profile_timestamp: 0,
            position_timestamp: 0,
            segment_timestamp: segment_ts,
            hda_timestamp: hda_ts,
            status_timestamp: 0,
            last_update: segment_ts.max(hda_ts),
            pt_timeout: false,
            alt_timeout: false,
            hda_size: 0,
            status_size: 0,
            metric: true,
        }
    }

    fn segment(bytes: [u8; 8]) -> Values {
        (1..=8)
            .map(|i| (format!("BYTE_{i}").into(), f64::from(bytes[i - 1])))
            .collect()
    }

    #[test]
    fn ev4_stock_navigation_is_available_from_its_hda_info() {
        for (ev4, expected) in [(true, true), (false, false)] {
            let mut navigation = Navigation::with(false, ev4, 3., 1, false);
            let mut output = Output::default();
            navigation
                .update_events(&input(None, 0, 1_000), &mut output, false)
                .expect("update");
            assert_eq!(output.available, expected, "ev4 {ev4}");
        }
    }

    #[test]
    fn ev4_padding_does_not_renew_the_route() {
        // BYTE_3 bit 6 is route state 1 (on route).
        let route = segment([0, 0, 0x40, 0, 0, 0, 0, 0]);
        let padding = segment([0xff; 8]);
        for (ev4, expected) in [(true, 100), (false, 200)] {
            let mut navigation = Navigation::with(false, ev4, 3., 1, false);
            navigation
                .update_route(&input(Some(&route), 100, 0))
                .expect("route");
            assert_eq!(navigation.route_state, 1);
            navigation
                .update_route(&input(Some(&padding), 200, 0))
                .expect("padding");
            assert_eq!(navigation.segment_timestamp, expected, "ev4 {ev4}");
        }
    }
}
