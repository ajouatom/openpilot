//! The N16 navigation input: the next maneuver as a Valhalla command one-hot
//! plus its distance. Port of carrot's `selfdrive/navigation/helpers.py`
//! (`valhalla_type`, `navigation_input`).

/// Valhalla's command count; the input has one more slot for the distance.
pub const VALHALLA_COMMAND_COUNT: usize = 46;
pub const NAVIGATION_LEN: usize = VALHALLA_COMMAND_COUNT + 1;

/// Mapbox step type/modifier to the Valhalla enum the model was trained on.
pub fn valhalla_type(maneuver: &str, modifier: &str) -> Option<usize> {
    let sided = match maneuver {
        "depart" => Some((1, 2, 3)),
        "arrive" => Some((4, 5, 6)),
        "on ramp" => Some((17, 18, 19)),
        "off ramp" => Some((20, 20, 21)),
        "fork" => Some((22, 23, 24)),
        "roundabout" | "rotary" | "roundabout turn" => Some((26, 26, 26)),
        "exit roundabout" | "exit rotary" => Some((27, 27, 27)),
        "notification" => Some((28, 28, 28)),
        "turn" | "continue" | "new name" | "end of road" => None,
        // Valhalla gives no maneuver at a merge, so the model never saw one.
        _ => return None,
    };
    if let Some((straight, right, left)) = sided {
        return Some(if modifier.contains("right") {
            right
        } else if modifier.contains("left") {
            left
        } else {
            straight
        });
    }
    Some(match modifier {
        "straight" => 8,
        "slight right" => 9,
        "right" => 10,
        "sharp right" => 11,
        "uturn" => 13,
        "sharp left" => 14,
        "left" => 15,
        "slight left" => 16,
        _ => 8,
    })
}

/// One maneuver of `navInstruction.allManeuvers`.
pub struct Maneuver<'a> {
    pub distance: f32,
    pub kind: &'a str,
    pub modifier: &'a str,
}

/// The first maneuver ahead (after `traveled` metres since the instruction)
/// that is a turn-like command: its one-hot and distance / 100.
pub fn navigation_input<'a>(
    maneuvers: impl IntoIterator<Item = Maneuver<'a>>,
    traveled: f32,
) -> [f32; NAVIGATION_LEN] {
    let mut navigation = [0.; NAVIGATION_LEN];
    for maneuver in maneuvers {
        let distance = maneuver.distance - traveled;
        if let Some(command) = valhalla_type(maneuver.kind, maneuver.modifier) {
            if command > 6 && distance.is_finite() && distance >= 0. {
                navigation[command] = 1.;
                navigation[NAVIGATION_LEN - 1] = distance / 100.;
                break;
            }
        }
    }
    navigation
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m<'a>(distance: f32, kind: &'a str, modifier: &'a str) -> Maneuver<'a> {
        Maneuver { distance, kind, modifier }
    }

    #[test]
    fn matches_carrot_helpers() {
        assert_eq!(valhalla_type("turn", "left"), Some(15));
        assert_eq!(valhalla_type("turn", "weird"), Some(8));
        assert_eq!(valhalla_type("off ramp", "slight right"), Some(20));
        assert_eq!(valhalla_type("fork", "left"), Some(24));
        assert_eq!(valhalla_type("merge", "left"), None);
        assert_eq!(valhalla_type("unknown", "left"), None);
        // depart/arrive (<= 6) are skipped; a passed maneuver too.
        let input = navigation_input(
            [m(50., "depart", "left"), m(80., "turn", "right"), m(300., "turn", "left")],
            100.,
        );
        assert_eq!(input[15], 1.);
        assert_eq!(input[10], 0.);
        assert!((input[46] - 2.).abs() < 1e-6);
        assert_eq!(navigation_input([m(f32::NAN, "turn", "left")], 0.), [0.; NAVIGATION_LEN]);
    }
}
