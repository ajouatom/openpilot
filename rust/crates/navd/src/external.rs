//! When navd steps aside for an external navigation app.
//!
//! navd.py steps aside whenever `carrotMan.activeCarrot > 0`. carrot_serv
//! uses that one number for two things (its own comment: 1 CarrotMan active,
//! 2 SDI active, 3 speed decel, 4 section, 5 bump, 6 speed limit): 1 and 2
//! mean an app is connected, 3 to 6 only say why carrot is slowing down. Those
//! come from the car's own navigation over CAN too (speed bump 5, section 4,
//! school zone 6, rear camera 3) with no app at all, and then navd.py blanked
//! its own Mapbox guidance and, each time the reason went back to 0, cleared
//! and recomputed the route. On drive 00000ccb that left the last 106 s
//! without a single instruction and missed the turn home.
//!
//! An app is connected when carrot names its owner (`naviOwner`) or reports
//! 1 or 2 itself; a speed reason alone is not an app.

/// Whether navd leaves guidance to an external app this tick.
pub fn external_navigation(alive_and_valid: bool, active_carrot: i32, has_owner: bool) -> bool {
    alive_and_valid && active_carrot > 0 && (has_owner || active_carrot <= 2)
}

#[cfg(test)]
mod tests {
    use super::external_navigation;

    #[test]
    fn an_app_owning_guidance_keeps_navd_aside_whatever_the_speed_reason() {
        for reason in 1..=6 {
            assert!(external_navigation(true, reason, true));
        }
    }

    #[test]
    fn carrot_reporting_its_app_states_keeps_navd_aside_without_an_owner() {
        assert!(external_navigation(true, 1, false));
        assert!(external_navigation(true, 2, false));
    }

    #[test]
    fn a_speed_reason_from_the_car_alone_does_not_blank_navd() {
        // 3 rear camera, 4 section, 5 speed bump, 6 school zone / limit.
        for reason in 3..=6 {
            assert!(!external_navigation(true, reason, false));
        }
    }

    #[test]
    fn nothing_active_or_a_stale_carrot_man_is_not_external() {
        assert!(!external_navigation(true, 0, false));
        assert!(!external_navigation(true, 0, true));
        assert!(!external_navigation(false, 1, true));
    }
}
