use super::{EventType, Input, Navigation, Output, Profile};
use crate::brands::hyundai::Error;

impl Navigation {
    pub fn update_events(
        &mut self,
        input: &Input<'_>,
        output: &mut Output,
        camera: bool,
    ) -> Result<bool, Error> {
        output.bump_distance = 0.;
        output.school_zone = false;
        output.active = false;
        output.section_active = false;
        output.speed = 0.;
        if self.wrapped {
            self.pv5_section(input)?;
        }
        let hda = self.ev4 && input.hda_timestamp > 0;
        self.available =
            self.available || input.profile_timestamp > 0 || hda || self.speed_zone_active;
        output.available = self.available;
        self.camera_target = None;
        let camera_speed = if camera { output.speed_limit } else { 0. };
        if camera_speed != self.camera_speed {
            self.camera_status_target = if camera_speed > 0. {
                Some(self.total_distance + camera_speed * self.distance_time)
            } else {
                None
            };
            self.camera_speed = camera_speed;
        }
        let position_seen = input.position_timestamp > 0;
        let position_age = i128::from(input.last_update) - i128::from(input.position_timestamp);
        let recent = position_seen && (0..=1_000_000_000).contains(&position_age);
        let range = if recent {
            input
                .position
                .and_then(|data| data.get("POS_RANGE_AVG_SPEED"))
                .copied()
                .unwrap_or(0.)
                .trunc()
        } else {
            0.
        };
        let section = range > 0. && range < 511.;
        self.update_route(input)?;
        let controlled_access = self.controlled_access(input);
        if controlled_access {
            self.clear_school();
        }
        if self.mode == 0 && !self.school_control {
            return Ok(false);
        }
        if let Some(data) = input.profile {
            let timestamp = input.profile_timestamp;
            if timestamp > self.profile_timestamp {
                self.profile_timestamp = timestamp;
                let profile = Profile::decode(data)?;
                if profile.profile_type == 16
                    && !self.wrapped
                    && timestamp > self.route_reset_timestamp
                {
                    self.camera_policy.add_preview(
                        profile.value,
                        f64::from(profile.offset),
                        self.total_distance,
                    );
                }
                if let Some((kind, speed, subtype)) = profile.classify() {
                    if timestamp > self.route_reset_timestamp {
                        match kind {
                            EventType::SpeedZone => {
                                if !self.wrapped {
                                    if self.profile_allowed(EventType::Camera, profile)
                                        && speed > 30.
                                    {
                                        self.speed_zone_active = true;
                                        self.speed_zone_speed = speed;
                                    }
                                    if self.school_control {
                                        if speed == 30.
                                            && camera
                                            && output.speed_limit == 30.
                                            && !controlled_access
                                        {
                                            self.school_active = true;
                                            self.school_start_distance = self.total_distance;
                                            self.school_uses_status = true;
                                        } else {
                                            self.clear_school();
                                        }
                                    }
                                }
                            }
                            EventType::Camera | EventType::Bump => {
                                if self.profile_allowed(kind, profile)
                                    && (!controlled_access
                                        || (kind != EventType::Bump
                                            && !(kind == EventType::Camera && speed == 30.)))
                                {
                                    self.add_event(kind, speed, subtype, profile.offset);
                                }
                            }
                        }
                    }
                }
            }
        }
        if position_seen {
            if !section || self.mode == 0 || (output.speed_limit > 0. && output.speed_limit <= 30.)
            {
                self.clear_speed_zone();
            } else if output.speed_limit > 30. && output.speed_limit < 255. {
                self.speed_zone_active = true;
                self.speed_zone_speed = output.speed_limit;
            }
        }
        let minimum = self.total_distance - 30.;
        self.events.retain(|event| {
            event.target >= minimum
                && (!controlled_access
                    || (event.kind != EventType::Bump
                        && !(event.kind == EventType::Camera && event.speed == 30.)))
        });
        let mut status_event = self.camera_event.clone().map(|cached| {
            self.events
                .iter()
                .find(|event| event.id == cached.id)
                .cloned()
                .unwrap_or(cached)
        });
        if self.wrapped {
            if status_event
                .as_ref()
                .is_some_and(|cached| !self.events.iter().any(|event| event.id == cached.id))
            {
                status_event = None;
            }
            if status_event.is_none() && camera {
                status_event = self
                    .events
                    .iter()
                    .find(|event| {
                        event.kind == EventType::Camera
                            && event.speed == camera_speed
                            && event.target > self.total_distance
                            && event.target
                                <= self.camera_status_target.unwrap_or(self.total_distance) + 40.
                    })
                    .cloned();
            }
            self.camera_event = status_event;
        } else if camera {
            if let Some(cached) = &status_event {
                if cached.speed != camera_speed {
                    self.events.retain(|event| event.id != cached.id);
                    status_event = None;
                }
            }
            if status_event.is_none() {
                status_event = self
                    .events
                    .iter()
                    .find(|event| {
                        event.kind == EventType::Camera
                            && event.speed == camera_speed
                            && event.target >= self.total_distance - 30.
                            && event.target
                                <= self.camera_status_target.unwrap_or(self.total_distance) + 40.
                    })
                    .cloned();
            }
            self.camera_event = status_event;
        } else if let Some(cached) = status_event {
            self.events.retain(|event| event.id != cached.id);
            self.camera_event = None;
        }
        let bump = self.events.iter().find(|event| {
            event.target > self.total_distance && event.kind == EventType::Bump && !event.cancelled
        });
        if let Some(event) = bump {
            output.bump_distance = event.target - self.total_distance;
        }
        let has_bump = bump.is_some();
        if self.speed_zone_active && !self.wrapped && !position_seen && !camera {
            self.clear_speed_zone();
        }
        if self.school_active
            && ((self.school_uses_status && (!camera || output.speed_limit != 30.))
                || self.total_distance - self.school_start_distance >= 1000.)
        {
            self.clear_school();
        }
        if self.school_control && self.school_active && !controlled_access {
            output.school_zone = true;
            output.speed_limit = 30.;
            if self.mode != 0 {
                output.active = true;
                output.speed = 30.;
            }
            return Ok(false);
        }
        if self.mode != 0 && self.speed_zone_active {
            output.active = true;
            output.section_active = true;
            output.speed = self.speed_zone_speed;
        }
        let target = if self.wrapped {
            if camera {
                self.camera_event.clone()
            } else {
                None
            }
        } else if camera {
            self.camera_event.clone()
        } else {
            self.events
                .iter()
                .find(|event| event.kind == EventType::Camera && event.target > self.total_distance)
                .cloned()
        };
        let mut has_camera = false;
        if let Some(event) = target {
            self.camera_target = Some(event.target);
            output.speed_limit = event.speed;
            output.active = true;
            if output.speed <= 0. {
                output.speed = event.speed;
            }
            has_camera = true;
        }
        if has_bump {
            output.active = true;
        }
        Ok(has_camera)
    }
}
