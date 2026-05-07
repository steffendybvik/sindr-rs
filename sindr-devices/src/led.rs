//! LED colour presets.
//!
//! Maps common LED colours to their typical forward voltage, then derives
//! DiodeParams via the Shockley equation so LEDs can be stamped as nonlinear
//! diodes in any MNA solver.

use crate::diode::DiodeParams;

/// Common LED colour variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedColor {
    Red,
    Green,
    Blue,
    Yellow,
    White,
}

impl LedColor {
    /// Typical forward voltage at 20 mA for this colour.
    pub fn forward_voltage(self) -> f64 {
        match self {
            LedColor::Red    => 1.8,
            LedColor::Green  => 2.2,
            LedColor::Blue   => 3.2,
            LedColor::Yellow => 2.0,
            LedColor::White  => 3.0,
        }
    }

    /// Parse from a lowercase string. Unknown strings default to `Red`.
    pub fn from_str(s: &str) -> Self {
        match s {
            "green"  => LedColor::Green,
            "blue"   => LedColor::Blue,
            "yellow" => LedColor::Yellow,
            "white"  => LedColor::White,
            _        => LedColor::Red,
        }
    }
}

/// `DiodeParams` for a given LED colour.
pub fn led_params(color: LedColor) -> DiodeParams {
    DiodeParams::led(color.forward_voltage())
}

/// `DiodeParams` from a colour name string. Unknown strings default to red.
pub fn led_params_from_str(color: &str) -> DiodeParams {
    led_params(LedColor::from_str(color))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_colors_have_correct_vf() {
        assert_eq!(LedColor::Red.forward_voltage(), 1.8);
        assert_eq!(LedColor::Blue.forward_voltage(), 3.2);
    }

    #[test]
    fn unknown_string_defaults_to_red() {
        assert_eq!(LedColor::from_str("ultraviolet"), LedColor::Red);
    }

    #[test]
    fn led_params_is_positive_and_ordered() {
        let red = led_params(LedColor::Red);
        let blue = led_params(LedColor::Blue);
        assert!(red.is > 0.0);
        assert!(blue.is > 0.0);
        // Higher Vf → smaller Is
        assert!(red.is > blue.is);
    }

    #[test]
    fn from_str_roundtrip() {
        for (s, expected) in &[
            ("red", LedColor::Red),
            ("green", LedColor::Green),
            ("blue", LedColor::Blue),
            ("yellow", LedColor::Yellow),
            ("white", LedColor::White),
        ] {
            assert_eq!(LedColor::from_str(s), *expected);
        }
    }
}
