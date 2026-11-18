//! carrot's `tools/jetlink/test_blinker_boundary.py` (BoundaryIndicator part).
use super::{boundary::BoundaryIndicator, lane::MANUAL_INDICATOR_MIN_SPEED};
use crate::types::{Car, Model};

fn scene(direction: u8) -> (Car, Model) {
    let car = Car {
        v_ego: 15.,
        left_lane_line: 10,
        right_lane_line: 10,
        steering_pressed: true,
        steering_torque: if direction == 1 { 300. } else { -300. },
        ..Car::default()
    };
    let model = Model {
        lane_line_probs: [0.9; 4],
        ..Model::default()
    };
    (car, model)
}

fn prepare(car: &mut Car, model: &mut Model, now: f64, left: f64, right: f64) {
    car.lane_line_info_mono_time = (now * 1e9) as u64;
    model.lane_lines[1] = vec![left; 9];
    model.lane_lines[2] = vec![right; 9];
    model.lane_lines[0] = vec![left - 3.6; 9];
    model.lane_lines[3] = vec![right + 3.6; 9];
    model.road_edges = [vec![left - 7.2; 9], vec![right + 7.2; 9]];
}

fn drive(
    b: &mut BoundaryIndicator,
    car: &mut Car,
    model: &mut Model,
    direction: u8,
    enabled: bool,
    valid: bool,
    busy: bool,
) -> Vec<(f64, u8, f64)> {
    let mut seen = Vec::new();
    for i in 0..65 {
        let t = 10. + f64::from(i) * 0.05;
        let y = 1.8 - f64::from((i - 30).max(0)) * 0.04;
        model.frame_id = i as u32;
        let (left, right) = if direction == 1 {
            (-y, 3.6 - y)
        } else {
            (-(3.6 - y), y)
        };
        prepare(car, model, t, left, right);
        b.update(t, car, model, enabled, valid, busy);
        if b.request_direction != 0 {
            seen.push((t, b.request_direction, b.request_time));
        }
    }
    seen
}

fn manual() -> BoundaryIndicator {
    BoundaryIndicator::new(None, super::lane::adjacent_lane, MANUAL_INDICATOR_MIN_SPEED)
}

#[test]
fn both_directions_need_real_driver_motion_and_emit_one_edge() {
    for direction in [1, 2] {
        let mut b = BoundaryIndicator::default();
        let (mut car, mut model) = scene(direction);
        let seen = drive(&mut b, &mut car, &mut model, direction, true, true, false);
        assert!(!seen.is_empty(), "direction {direction}");
        assert!(seen.iter().all(|v| v.1 == direction));
        assert!(seen.iter().all(|v| v.2 == seen[0].2), "one edge");
        assert_eq!(b.filter_lamp(direction), 0);
    }
}

#[test]
fn slow_manual_lane_crossing_reaches_the_gate() {
    for direction in [1, 2] {
        for (kph, expected) in [
            (0., false),
            (4.9, false),
            (5., true),
            (10., true),
            (26.4, true),
        ] {
            let (mut car, mut model) = scene(direction);
            car.v_ego = kph / 3.6;
            let seen = drive(
                &mut manual(),
                &mut car,
                &mut model,
                direction,
                true,
                true,
                false,
            );
            assert_eq!(!seen.is_empty(), expected, "{kph} km/h dir {direction}");
            let standard = drive(
                &mut BoundaryIndicator::default(),
                &mut car,
                &mut model,
                direction,
                true,
                true,
                false,
            );
            assert!(standard.is_empty(), "standard 30 km/h profile");
        }
    }
}

#[test]
fn driver_vehicle_and_existing_signal_gates() {
    let cases: [fn(&mut Car); 12] = [
        |c| c.steering_pressed = false,
        |c| c.steering_torque = -300.,
        |c| c.steering_torque = f64::NAN,
        |c| c.v_ego = 4.9 / 3.6,
        |c| c.v_ego = f64::NAN,
        |c| c.drive = false,
        |c| c.can_valid = false,
        |c| c.trailer_connected = true,
        |c| c.steer_fault_temporary = true,
        |c| c.left_blinker = true,
        |c| c.right_blinker = true,
        |c| c.blinker_lever = 1,
    ];
    for (index, case) in cases.iter().enumerate() {
        let (mut car, mut model) = scene(1);
        case(&mut car);
        let seen = drive(
            &mut BoundaryIndicator::default(),
            &mut car,
            &mut model,
            1,
            true,
            true,
            false,
        );
        assert!(seen.is_empty(), "case {index}");
    }
}

#[test]
fn disabled_invalid_busy_do_not_request() {
    for (enabled, valid, busy) in [
        (false, true, false),
        (true, false, false),
        (true, true, true),
    ] {
        let (mut car, mut model) = scene(1);
        let seen = drive(
            &mut BoundaryIndicator::default(),
            &mut car,
            &mut model,
            1,
            enabled,
            valid,
            busy,
        );
        assert!(seen.is_empty());
    }
}

#[test]
fn old_visible_line_is_not_enough() {
    for probs in [[0.9, 0.01, 0.9, 0.9], [0.9, f64::NAN, 0.9, 0.9]] {
        let (mut car, mut model) = scene(1);
        model.lane_line_probs = probs;
        let seen = drive(
            &mut BoundaryIndicator::default(),
            &mut car,
            &mut model,
            1,
            true,
            true,
            false,
        );
        assert!(seen.is_empty());
    }
}

#[test]
fn inhibit_survives_disable_dark_flash_and_restart_journal() {
    let mut first = BoundaryIndicator::default();
    let (mut car, mut model) = scene(1);
    let seen = drive(&mut first, &mut car, &mut model, 1, true, true, false);
    let stamp = seen[0].2;
    let restarted = BoundaryIndicator::new(
        Some((stamp, 1, None)),
        super::lane::adjacent_lane,
        30. / 3.6,
    );
    for mut b in [first, restarted] {
        let (left, right) = (model.lane_lines[1][0], model.lane_lines[2][0]);
        for i in 0..300 {
            let now = stamp + 0.2 + f64::from(i) * 0.05;
            car.left_blinker = (i % 16) < 8;
            prepare(&mut car, &mut model, now, left, right);
            b.update(now, &car, &model, false, true, false);
            assert_eq!(b.filter_lamp(1), 0);
            assert_eq!(b.request_direction, 0);
        }
        car.left_blinker = false;
        for i in 0..23 {
            let now = stamp + 16. + f64::from(i) * 0.05;
            prepare(&mut car, &mut model, now, left, right);
            b.update(now, &car, &model, false, true, false);
        }
        assert_eq!(b.filter_lamp(1), 1);
    }
}

// carrot's `test_indicator_reconstruction.py` scene.
fn reconstruction() -> (Car, Model) {
    let xs: Vec<f64> = (0..13).map(|i| f64::from(i) * 5.).collect();
    let car = Car {
        v_ego: 15.,
        steering_angle_deg: -5.,
        left_lane_line: 10,
        lane_line_info_mono_time: 10_000_000_000,
        ..Car::default()
    };
    let model = Model {
        lane_lines: [-5.4, -1.8, 1.8, 5.4].map(|y| vec![y; 13]),
        lane_line_x: std::array::from_fn(|_| xs.clone()),
        road_edges: [vec![-8.; 13], vec![8.; 13]],
        lane_line_probs: [0.9, 0.9, 0.10, 0.9],
        position_t: vec![0., 1., 1.5, 2., 2.5, 3., 4.],
        position_x: vec![0., 10., 15., 20., 25., 30., 40.],
        position_y: vec![0., 0.1, 0.7, 1.5, 2., 2.4, 3.],
        desire_state: [1., 0., 0., 0., 0., 0., 0., 0.],
        ..Model::default()
    };
    (car, model)
}

fn observe(
    detector: &mut super::observer::ModelIndicator,
    isolation: &mut BoundaryIndicator,
    now: f64,
    car: &Car,
    model: &Model,
    path_enabled: bool,
    manual_turn_enabled: bool,
    valid: bool,
    busy: bool,
    yaw_rate: Option<f64>,
    lateral_active: bool,
) {
    detector.update(
        super::observer::ObserverInput {
            now,
            car,
            model,
            valid,
            lateral_active,
            desire_enabled: false,
            path_enabled,
            busy,
            manual_edge: 0,
            command_curvature: Some(0.001),
            feedback: None,
            manual_turn_enabled,
            yaw_rate,
        },
        isolation,
    );
}

#[test]
fn path_signal_accepts_ten_percent_and_owns_feedback_before_lamp() {
    let (mut car, mut model) = reconstruction();
    let mut detector = super::observer::ModelIndicator::default();
    let mut isolation = BoundaryIndicator::default();
    for i in 0..10 {
        let now = 10. + f64::from(i) * 0.05;
        model.frame_id = i;
        car.lane_line_info_mono_time = (now * 1e9) as u64;
        observe(
            &mut detector,
            &mut isolation,
            now,
            &car,
            &model,
            true,
            false,
            true,
            false,
            None,
            true,
        );
    }
    let active = detector.active.as_ref().expect("observed");
    assert_eq!((active.direction(), active.source()), (2, "model_path"));
    assert_eq!(isolation.filter_lamp(2), 0);
}

fn manual_turn(direction: u8, fault: &str) -> Vec<(f64, u8)> {
    let (mut car, mut model) = reconstruction();
    let mut detector = super::observer::ModelIndicator::default();
    let mut isolation = BoundaryIndicator::default();
    let sign = if direction == 1 { 1. } else { -1. };
    let mut history = Vec::new();
    for i in 0..65 {
        let now = 10. + f64::from(i) * 0.05;
        model.frame_id = i;
        car.v_ego = 5.;
        car.steering_pressed = i >= 25;
        car.steering_torque = if car.steering_pressed {
            400. * sign
        } else {
            0.
        };
        car.steering_angle_deg = (f64::from(i.saturating_sub(25)) * 8.).min(150.) * sign;
        let mut yaw = Some(if i >= 29 {
            -12f64.to_radians() * sign
        } else {
            0.
        });
        model.lane_line_probs = [0.; 4];
        match fault {
            "no_driver" => car.steering_pressed = false,
            "brief_torque" => car.steering_pressed = i == 25,
            "wrong_yaw" => yaw = yaw.map(|y| -y),
            "no_rotation" => yaw = Some(0.),
            "small_angle" => car.steering_angle_deg = car.steering_angle_deg.abs().min(40.) * sign,
            "moderate_turn" => {
                car.steering_angle_deg = (20. + car.steering_angle_deg.abs().min(22.5)) * sign;
            }
            "held_angle" => car.steering_angle_deg = 100. * sign,
            "park" | "reverse" => car.drive = false,
            "stale_pose" => yaw = None,
            "nan" => yaw = Some(f64::NAN),
            "stationary" => car.v_ego = 0.,
            "fast" => car.v_ego = 61. / 3.6,
            "stalk" => car.blinker_lever = 2,
            "lamps" => car.right_blinker = true,
            "duplicate_frame" => model.frame_id = 0,
            "slow_relabel" | "fast_relabel" => {
                car.v_ego = if fault == "slow_relabel" { 8. } else { 35. } / 3.6;
                isolation.relabel_until = now + 2.;
            }
            _ => {}
        }
        isolation.update(now, &car, &model, false, true, false);
        observe(
            &mut detector,
            &mut isolation,
            now,
            &car,
            &model,
            false,
            fault != "disabled",
            fault != "invalid",
            fault == "busy",
            yaw,
            false,
        );
        if let Some(active) = &detector.active {
            assert_eq!(active.source(), "manual_turn");
            history.push((now, active.direction()));
        }
    }
    history
}

#[test]
fn manual_turn_starts_without_atc_lanes_or_lateral_activation() {
    for direction in [1, 2] {
        let history = manual_turn(direction, "");
        assert!(!history.is_empty());
        assert!(history.iter().all(|h| h.1 == direction));
    }
}

#[test]
fn manual_turn_counterexamples_never_request() {
    for fault in [
        "no_driver",
        "brief_torque",
        "wrong_yaw",
        "no_rotation",
        "small_angle",
        "held_angle",
        "park",
        "reverse",
        "stale_pose",
        "nan",
        "stationary",
        "fast",
        "stalk",
        "lamps",
        "duplicate_frame",
        "invalid",
        "busy",
        "disabled",
    ] {
        assert!(manual_turn(1, fault).is_empty(), "{fault}");
    }
}

#[test]
fn moderate_turn_and_low_speed_relabel() {
    for direction in [1, 2] {
        assert!(!manual_turn(direction, "moderate_turn").is_empty());
    }
    assert!(!manual_turn(1, "slow_relabel").is_empty());
    // carrot 569f3191e: a junction relabel no longer erases the steering onset
    // at any speed (its Python test flipped the same assertion).
    assert!(!manual_turn(1, "fast_relabel").is_empty());
}
