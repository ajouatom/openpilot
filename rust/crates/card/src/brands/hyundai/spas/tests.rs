//! Port of carrot's `opendbc/car/hyundai/tests/test_spas.py`.
use super::*;

const LEFT: Option<Direction> = Some(Direction::Left);

fn stock(s: &mut Session, t: f64, count: u8) -> f64 {
    for i in 0..count {
        for (address, _) in SPAS {
            let m = frame(address, i, None, s.bus);
            s.observe(m.address, m.bus, &m.data, t + f64::from(i) * 0.02);
        }
    }
    t + f64::from(count - 1) * 0.02
}

struct Step<'a> {
    healthy: bool,
    request: Option<&'a str>,
    direction: Option<Direction>,
    driver_override: bool,
}
impl Default for Step<'_> {
    fn default() -> Self {
        Self {
            healthy: true,
            request: Some("a"),
            direction: LEFT,
            driver_override: false,
        }
    }
}
fn step(s: &mut Session, t: f64, kw: Step<'_>) -> Vec<Frame> {
    s.update(
        t,
        Update {
            healthy: kw.healthy,
            request: kw.request,
            direction: kw.direction,
            duration: 300.,
            driver_override: kw.driver_override,
        },
    )
}
fn none() -> Step<'static> {
    Step {
        request: None,
        ..Step::default()
    }
}
fn reply(s: &mut Session, t: f64, body: &[u8]) {
    let mut data = vec![body.len() as u8];
    data.extend_from_slice(body);
    data.resize(8, 0);
    s.observe(RESPONSE, s.bus, &data, t);
}
fn opened(bus: u8) -> (Session, f64) {
    let mut s = Session::new(bus);
    let t = stock(&mut s, 10.0, 10);
    step(&mut s, t, none());
    assert_eq!(step(&mut s, t + 0.01, Step::default())[0].data, EXTENDED);
    reply(&mut s, t + 0.02, &[0x50, 0x03]);
    assert_eq!(step(&mut s, t + 0.03, Step::default())[0].data, DISABLE);
    reply(&mut s, t + 0.04, &[0x68, 0x03]);
    assert!(step(&mut s, t + 0.05, Step::default()).is_empty());
    assert!(!step(&mut s, t + 0.16, Step::default()).is_empty());
    assert_eq!(s.state, State::Active);
    (s, t + 0.16)
}
fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}

#[test]
fn supplied_vectors_match() {
    assert_eq!(
        hex(&frame(0x16a, 0, LEFT, 0).data),
        "db65000000000000000000000000000060000000000000000000000000000000"
    );
    assert_eq!(
        hex(&frame(0x16a, 0, Some(Direction::Right), 0).data),
        "848c000000000000000000000000000080000000000000000000000000000000"
    );
}

#[test]
fn bus_contract_and_native_frames() {
    for bus in [0u8, 1, 4, 5] {
        let full: BTreeMap<u32, usize> = SPAS.into_iter().collect();
        assert!(compatible(&BTreeMap::from([(bus, full.clone())]), bus));
        assert!(!compatible(
            &BTreeMap::from([(bus, BTreeMap::from([(0x165, 16), (0x16a, 32)]))]),
            bus
        ));
        assert!(!compatible(&BTreeMap::from([(bus + 1, full)]), bus));
        let (mut s, t) = opened(bus);
        let messages = step(&mut s, t + 0.021, Step::default());
        assert_eq!(
            messages.iter().map(|m| m.address).collect::<Vec<_>>(),
            vec![0x165, 0x16a]
        );
        assert!(messages.iter().all(|m| m.bus == bus
            && u16::from_le_bytes([m.data[0], m.data[1]]) == checksum(m.address, &m.data)));
    }
}

#[test]
fn incompatible_stock_never_requests_disable() {
    for failure in [
        "wrong_bus",
        "wrong_dlc",
        "bad_crc",
        "repeated_counter",
        "stale",
        "insufficient",
    ] {
        let mut s = Session::new(0);
        for i in 0..10u8 {
            for (a, _) in SPAS {
                let m = frame(
                    a,
                    if failure == "repeated_counter" { 0 } else { i },
                    None,
                    if failure == "wrong_bus" { 1 } else { 0 },
                );
                let mut data = m.data.clone();
                if failure == "wrong_dlc" {
                    data.pop();
                }
                if failure == "bad_crc" {
                    data[0] ^= 1;
                }
                if failure == "insufficient" && i > 7 {
                    continue;
                }
                s.observe(a, m.bus, &data, 10. + f64::from(i) * 0.02);
            }
        }
        let t = if failure == "stale" { 11.0 } else { 10.19 };
        step(&mut s, t, none());
        assert!(
            step(&mut s, t + 0.01, Step::default()).is_empty(),
            "{failure}"
        );
    }
}

#[test]
fn missing_or_negative_extended_ack_never_sends_spas() {
    for body in [&[0x7f, 0x10, 0x22][..], &[0x50, 0x01], &[0x68, 0x03]] {
        let mut s = Session::new(0);
        let t = stock(&mut s, 10.0, 10);
        step(&mut s, t, none());
        step(&mut s, t + 0.01, Step::default());
        reply(&mut s, t + 0.02, body);
        let mut messages = Vec::new();
        for i in 1..50 {
            messages.extend(step(
                &mut s,
                t + 0.02 + f64::from(i) * 0.01,
                Step::default(),
            ));
        }
        assert!(!messages
            .iter()
            .any(|m| size(m.address).is_some() || m.data == DISABLE));
        assert!(s.state.restoring());
    }
}

#[test]
fn stock_sender_must_be_quiet() {
    let mut s = Session::new(0);
    let t = stock(&mut s, 10.0, 10);
    step(&mut s, t, none());
    step(&mut s, t + 0.01, Step::default());
    reply(&mut s, t + 0.02, &[0x50, 0x03]);
    step(&mut s, t + 0.03, Step::default());
    reply(&mut s, t + 0.04, &[0x68, 0x03]);
    step(&mut s, t + 0.05, Step::default());
    for i in 1..45u32 {
        let now = t + 0.05 + f64::from(i) * 0.01;
        for (a, _) in SPAS {
            let m = frame(a, ((9 + i) % 256) as u8, None, 0);
            s.observe(a, 0, &m.data, now);
        }
        assert!(!step(&mut s, now, Step::default())
            .iter()
            .any(|m| size(m.address).is_some()));
    }
    assert_eq!(s.reason, "ecu_not_quiet");
}

#[test]
fn active_session_revoked_and_restores_on_every_failure() {
    for cause in [
        "unhealthy",
        "gap",
        "request",
        "direction",
        "driver",
        "stock",
        "deadline",
    ] {
        let (mut s, t) = opened(0);
        let mut kw = Step::default();
        let mut dt = 0.01;
        match cause {
            "unhealthy" => kw.healthy = false,
            "gap" => dt = 0.21,
            "request" => kw.request = Some("new"),
            "direction" => kw.direction = Some(Direction::Right),
            "driver" => kw.driver_override = true,
            "deadline" => s.set_started(t - 301.),
            _ => {
                let m = frame(0x16a, 10, None, 0);
                s.observe(0x16a, 0, &m.data, t + 0.001);
            }
        }
        assert_eq!(step(&mut s, t + dt, kw)[0].data, ENABLE, "{cause}");
        assert_eq!(s.state, State::RestoreEnable);
    }
}

#[test]
fn recovery_requires_reply_and_resumed_stock_and_no_replay() {
    let (mut s, t) = opened(0);
    step(&mut s, t + 0.01, none());
    reply(&mut s, t + 0.02, &[0x68, 0x00]);
    assert_eq!(step(&mut s, t + 0.03, none())[0].data, DEFAULT);
    reply(&mut s, t + 0.04, &[0x50, 0x01]);
    step(&mut s, t + 0.05, none());
    assert_eq!(s.state, State::RestoreStock);
    for i in 0..12u8 {
        let now = t + 0.06 + f64::from(i) * 0.02;
        for (a, _) in SPAS {
            let m = frame(a, 20 + i, None, 0);
            s.observe(a, 0, &m.data, now);
        }
        step(&mut s, now, none());
    }
    assert!(s.state == State::Idle && s.stock_resumed && s.restore_ack);
    step(
        &mut s,
        t + 0.30,
        Step {
            healthy: false,
            ..Step::default()
        },
    );
    assert!(step(&mut s, t + 0.31, Step::default()).is_empty());
}

#[test]
fn restore_is_bounded_and_cannot_rearm_after_failure() {
    let (mut s, t) = opened(0);
    step(&mut s, t + 0.01, none());
    let mut sent = Vec::new();
    for i in 1..900 {
        sent.extend(step(&mut s, t + 0.01 + f64::from(i) * 0.01, none()));
    }
    assert!(s.state == State::Failed && s.reason == "restore_unconfirmed");
    assert_eq!(sent.iter().filter(|m| m.data == ENABLE).count(), 2);
    stock(&mut s, t + 10., 10);
    assert!(step(
        &mut s,
        t + 10.2,
        Step {
            request: Some("new"),
            ..Step::default()
        }
    )
    .is_empty());
}

#[test]
fn fifty_hz_no_burst_after_delayed_update() {
    let (mut s, t) = opened(0);
    let mut out = Vec::new();
    for i in 1..100 {
        let now = t + f64::from(i) * 0.01;
        for m in step(&mut s, now, Step::default()) {
            if m.address == 0x16a {
                out.push((now, m));
            }
        }
    }
    for pair in out.windows(2) {
        assert!(pair[1].0 - pair[0].0 >= 0.019999);
        assert_eq!(pair[1].1.data[2], pair[0].1.data[2].wrapping_add(1));
    }
}

#[test]
fn no_stock_sender_is_absent_and_a_late_one_is_not() {
    let mut s = Session::new(0);
    assert!(s.stock_absent(5.), "never seen");
    let t = stock(&mut s, 5., 3);
    assert!(!s.stock_absent(t + 0.5));
    assert!(s.stock_absent(t + 1.0), "quiet for one second");
    // A garbage frame on a SPAS address does not count as a sender.
    s.observe(0x165, 0, &[0; 24], t + 2.);
    assert!(s.stock_absent(t + 2.));
    // Another bus is not the ECAN sender.
    let m = frame(0x16a, 0, None, 2);
    s.observe(m.address, m.bus, &m.data, t + 3.);
    assert!(s.stock_absent(t + 3.));
}

#[test]
fn spas_less_fingerprint_is_eligible_only_as_absent() {
    let none = |_: u32| None;
    assert!(absent_with(none) && !compatible_with(none));
    let both = |a: u32| size(a);
    assert!(compatible_with(both) && !absent_with(both));
    let one = |a: u32| (a == 0x165).then_some(24);
    assert!(
        !compatible_with(one) && !absent_with(one),
        "a partial SPAS bus is neither"
    );
}

#[test]
fn solo_trial_frames_match_the_panda_arming_rules() {
    // Panda arms on an all-zero SPAS1 (counter/CRC aside) and admits only the
    // SPAS2 direction byte 16 as 0x60 (left) or 0x80 (right).
    let arm = frame(0x165, 0, LEFT, 0);
    assert!(arm.data[3..].iter().all(|b| *b == 0));
    let left = frame(0x16a, 1, LEFT, 0);
    let right = frame(0x16a, 1, Some(Direction::Right), 0);
    for (f, byte) in [(left, 0x60), (right, 0x80)] {
        assert_eq!(f.data[16], byte);
        assert!(f.data[3..]
            .iter()
            .enumerate()
            .all(|(i, b)| i + 3 == 16 || *b == 0));
        assert_eq!(
            u16::from_le_bytes([f.data[0], f.data[1]]),
            checksum(0x16a, &f.data)
        );
    }
}

#[test]
fn solo_trial_leads_and_ends_with_zero_direction() {
    for (elapsed, expected) in [
        (0., None),
        (0.19, None),
        (0.2, LEFT),
        (1.5, LEFT),
        (2.79, LEFT),
        (2.8, None),
        (3.0, None),
    ] {
        assert_eq!(solo_direction(elapsed, 3.0, LEFT), expected, "{elapsed}");
    }
}

/// Runs `SoloDrive` at 100 Hz from `t0` to `t1`; returns (time, SPAS2 direction byte) per sent pair.
fn solo_run(
    solo: &mut SoloDrive,
    t0: f64,
    t1: f64,
    want: impl Fn(f64) -> Option<Direction>,
    allowed: impl Fn(f64) -> bool,
    started: &mut Vec<(f64, Direction)>,
) -> Vec<(f64, u8)> {
    let mut sent = Vec::new();
    let mut t = t0;
    while t < t1 {
        let (start, frames) = solo.update(t, want(t), allowed(t), 0);
        if let Some(side) = start {
            started.push((t, side));
        }
        for f in frames {
            if f.address == 0x16a {
                sent.push((t, f.data[16]));
            }
        }
        t += 0.01;
    }
    sent
}

#[test]
fn solo_drive_follows_the_lane_change_and_ends_with_a_zero_tail() {
    let mut solo = SoloDrive::default();
    let mut started = Vec::new();
    // Lane change 1.0 .. 6.0 s.
    let sent = solo_run(
        &mut solo,
        0.,
        9.,
        |t| (1. ..6.).contains(&t).then_some(Direction::Left),
        |_| true,
        &mut started,
    );
    assert_eq!(started.len(), 1);
    assert!((started[0].0 - 1.).abs() < 0.02);
    let lit: Vec<_> = sent.iter().filter(|(_, d)| *d == 0x60).collect();
    let zero: Vec<_> = sent.iter().filter(|(_, d)| *d == 0).collect();
    assert!(
        (240..=252).contains(&lit.len()),
        "50 Hz for 5 s: {}",
        lit.len()
    );
    assert!(lit.last().unwrap().0 < 6.0);
    assert!(
        (13..=17).contains(&zero.len()),
        "0.3 s tail: {}",
        zero.len()
    );
    assert!(zero.iter().all(|(t, _)| (6.0..6.35).contains(t)));
    assert!(!solo.busy());
}

#[test]
fn solo_drive_waits_out_the_cooldown_and_changes_side_through_a_tail() {
    let mut solo = SoloDrive::default();
    let mut started = Vec::new();
    let want = |t: f64| {
        if t < 3. {
            Some(Direction::Left)
        } else {
            Some(Direction::Right)
        }
    };
    let sent = solo_run(&mut solo, 0., 8., want, |_| true, &mut started);
    assert_eq!(
        started.iter().map(|s| s.1).collect::<Vec<_>>(),
        vec![Direction::Left, Direction::Right]
    );
    // Tail 3.0..3.3, cooldown until 4.8.
    assert!(started[1].0 >= 4.8 - 1e-9, "{}", started[1].0);
    assert!(
        !sent.iter().any(|(t, d)| (3.0..3.3).contains(t) && *d != 0),
        "only zeros in the tail"
    );
}

#[test]
fn solo_drive_stops_on_driver_input_and_after_its_limit() {
    let mut solo = SoloDrive::default();
    let mut started = Vec::new();
    // Driver uses the lever at 2 s: tail, and no new lease while not allowed.
    let sent = solo_run(
        &mut solo,
        0.,
        5.,
        |_| Some(Direction::Left),
        |t| t < 2.,
        &mut started,
    );
    assert_eq!(started.len(), 1);
    assert!(!sent.iter().any(|(t, d)| *t >= 2. && *d != 0));
    // A lane change that never completes ends at SOLO_DRIVE_MAX and does
    // not restart until it clears.
    let mut solo = SoloDrive::default();
    let mut started = Vec::new();
    let sent = solo_run(
        &mut solo,
        0.,
        40.,
        |_| Some(Direction::Right),
        |_| true,
        &mut started,
    );
    assert_eq!(started.len(), 1);
    assert!(!sent
        .iter()
        .any(|(t, d)| *t >= SOLO_DRIVE_MAX + 0.01 && *d != 0));
    let _ = solo_run(&mut solo, 40., 41., |_| None, |_| true, &mut started);
    let _ = solo_run(
        &mut solo,
        41.,
        42.,
        |_| Some(Direction::Right),
        |_| true,
        &mut started,
    );
    assert_eq!(started.len(), 2, "a new lane change starts again");
}
