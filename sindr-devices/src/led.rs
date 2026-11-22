//! LED colour presets.
//!
//! Maps common LED colours to their typical forward voltage, then derives
//! DiodeParams via the Shockley equation so LEDs can be stamped as nonlinear
//! diodes in any MNA solver.
//!
//! [`LedColor`] has the five original colours. Colour names, amber and
//! orange included, are read by [`forward_voltage_of`] and
//! [`color_in`], which ignore case.

use crate::diode::DiodeParams;

/// Common LED colour variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedColor {
    /// Red LED (~1.8 V typical forward voltage).
    Red,
    /// Green LED (~2.2 V typical forward voltage).
    Green,
    /// Blue LED (~3.2 V typical forward voltage).
    Blue,
    /// Yellow LED (~2.0 V typical forward voltage).
    Yellow,
    /// White LED (~3.0 V typical forward voltage).
    White,
}

impl LedColor {
    /// Typical forward voltage (V) at 20 mA for this colour.
    pub fn forward_voltage(self) -> f64 {
        match self {
            LedColor::Red => 1.8,
            LedColor::Green => 2.2,
            LedColor::Blue => 3.2,
            LedColor::Yellow => 2.0,
            LedColor::White => 3.0,
        }
    }

    /// Parse from a lowercase string. Unknown strings default to `Red`.
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Self {
        match s {
            "green" => LedColor::Green,
            "blue" => LedColor::Blue,
            "yellow" => LedColor::Yellow,
            "white" => LedColor::White,
            _ => LedColor::Red,
        }
    }
}

/// Every colour name sindr knows, lowercase, with its typical forward
/// voltage (V) at 20 mA. Amber (≈ 590 nm) and orange (≈ 605 nm) parts are
/// AlInGaP, like most red and yellow indicators, and sit between red and
/// green: about 2.0–2.1 V at 20 mA on common 3 mm and 5 mm datasheets.
pub const COLORS: &[(&str, f64)] = &[
    ("red", 1.8),
    ("orange", 2.0),
    ("amber", 2.05),
    ("yellow", 2.0),
    ("green", 2.2),
    ("blue", 3.2),
    ("white", 3.0),
];

/// The typical forward voltage (V) at 20 mA of the colour `name`, case
/// ignored; `None` for a name sindr doesn't know.
///
/// ```
/// use sindr_devices::led::forward_voltage_of;
///
/// assert_eq!(forward_voltage_of("Amber"), Some(2.05));
/// assert_eq!(forward_voltage_of("ultraviolet"), None);
/// ```
pub fn forward_voltage_of(name: &str) -> Option<f64> {
    let name = name.trim();
    COLORS
        .iter()
        .find(|(c, _)| c.eq_ignore_ascii_case(name))
        .map(|(_, vf)| *vf)
}

/// The first colour word in `text`, as its lowercase name: `"LED red
/// 3mm"` is `"red"`. Words are split at anything but letters and digits,
/// and compared without case.
///
/// ```
/// use sindr_devices::led::color_in;
///
/// assert_eq!(color_in("LED Amber 5mm"), Some("amber"));
/// assert_eq!(color_in("Green"), Some("green"));
/// assert_eq!(color_in("LTST-C190"), None);
/// ```
pub fn color_in(text: &str) -> Option<&'static str> {
    text.split(|c: char| !c.is_alphanumeric())
        .find_map(|word| COLORS.iter().find(|(c, _)| c.eq_ignore_ascii_case(word)))
        .map(|(c, _)| *c)
}

/// `DiodeParams` for a given LED colour.
pub fn led_params(color: LedColor) -> DiodeParams {
    DiodeParams::led(color.forward_voltage())
}

/// `DiodeParams` from a colour name, case ignored ([`COLORS`], amber and
/// orange included). Unknown names default to red.
pub fn led_params_from_str(color: &str) -> DiodeParams {
    DiodeParams::led(forward_voltage_of(color).unwrap_or(LedColor::Red.forward_voltage()))
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

    #[test]
    fn every_preset_has_its_named_colours_vf() {
        for (name, color) in [
            ("red", LedColor::Red),
            ("green", LedColor::Green),
            ("blue", LedColor::Blue),
            ("yellow", LedColor::Yellow),
            ("white", LedColor::White),
        ] {
            assert_eq!(forward_voltage_of(name), Some(color.forward_voltage()));
            assert_eq!(led_params(color).is, led_params_from_str(name).is, "{name}");
        }
        assert_eq!(LedColor::Green.forward_voltage(), 2.2);
        assert_eq!(LedColor::Yellow.forward_voltage(), 2.0);
        assert_eq!(LedColor::White.forward_voltage(), 3.0);
    }

    #[test]
    fn names_ignore_case_and_include_amber_and_orange() {
        assert_eq!(forward_voltage_of("RED"), Some(1.8));
        assert_eq!(forward_voltage_of("orange"), Some(2.0));
        let amber = led_params_from_str("Amber");
        let red = led_params_from_str("red");
        assert!(amber.is < red.is, "amber's Vf is above red's");
        for (name, vf) in COLORS {
            let p = led_params_from_str(name);
            // Each colour's preset passes through its Vf at 20 mA.
            let i = p.is * ((vf / (p.n * p.vt())).exp() - 1.0);
            assert!((i - 0.020).abs() < 1e-9, "{name}");
        }
        assert_eq!(color_in("LED red 3mm"), Some("red"));
        assert_eq!(color_in("Amber"), Some("amber"));
        assert_eq!(color_in(""), None);
    }
}
