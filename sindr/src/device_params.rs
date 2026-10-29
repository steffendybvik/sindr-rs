//! Effective device parameters for a [`CircuitElement`](crate::CircuitElement)'s fields.
//!
//! Every analysis that evaluates a nonlinear device (DC stamping, voltage
//! limiting, AC linearisation, transient stamping, result reporting) builds its
//! parameters here, so they all see the same temperature-scaled model as the
//! operating point that was solved.

use sindr_devices::bjt::BjtParams;
use sindr_devices::diode::{temperature_scale_is, DiodeParams};
use sindr_devices::led::led_params_from_str;
use sindr_devices::photodiode::PhotodiodeParams;
use sindr_devices::schottky::SchottkyParams;
use sindr_devices::zener::ZenerParams;

/// Nominal temperature (K) the default parameters are given at.
const T_NOM: f64 = 300.15;

/// Silicon bandgap (eV).
const EG_SI: f64 = 1.11;

/// IS temperature exponent: SPICE's XTI for diodes and for BJTs.
const XTI_DIODE: f64 = 2.0;
const XTI_BJT: f64 = 3.0;

/// `is` at nominal temperature, scaled to `temperature`.
fn scale_is(is: f64, temperature: f64, xti: f64) -> f64 {
    if (temperature - T_NOM).abs() > 1e-6 {
        temperature_scale_is(is, temperature, T_NOM, EG_SI, xti)
    } else {
        is
    }
}

/// Silicon diode at `temperature` (K).
pub(crate) fn diode(temperature: f64) -> DiodeParams {
    with_temperature(DiodeParams::silicon(), temperature)
}

/// LED of colour `color` at `temperature` (K).
pub(crate) fn led(color: &str, temperature: f64) -> DiodeParams {
    with_temperature(led_params_from_str(color), temperature)
}

/// Schottky diode at `temperature` (K), as the Shockley diode it is stamped as.
pub(crate) fn schottky(temperature: f64) -> DiodeParams {
    let schottky = SchottkyParams::default();
    with_temperature(
        DiodeParams {
            is: schottky.is,
            n: schottky.n,
            rs: 0.0,
            temperature: T_NOM,
        },
        temperature,
    )
}

/// Zener diode with breakdown voltage `vz` at `temperature` (K).
pub(crate) fn zener(vz: f64, temperature: f64) -> ZenerParams {
    let mut params = ZenerParams::new(vz);
    params.is = scale_is(params.is, temperature, XTI_DIODE);
    params
}

/// Photodiode at `temperature` (K). The dark current scales like a diode's;
/// responsivity is taken as temperature-independent.
pub(crate) fn photodiode(temperature: f64) -> PhotodiodeParams {
    let mut params = PhotodiodeParams::default();
    params.is = scale_is(params.is, temperature, XTI_DIODE);
    params
}

/// The dark diode of a photodiode at `temperature` (K).
pub(crate) fn photodiode_dark(temperature: f64) -> DiodeParams {
    let params = photodiode(temperature);
    DiodeParams {
        is: params.is,
        n: params.n,
        rs: 0.0,
        temperature,
    }
}

/// BJT with forward gain `bf` at `temperature` (K).
pub(crate) fn bjt(bf: f64, temperature: f64) -> BjtParams {
    let mut params = BjtParams::new(bf);
    params.is = scale_is(params.is, temperature, XTI_BJT);
    params.temperature = temperature;
    params
}

fn with_temperature(mut params: DiodeParams, temperature: f64) -> DiodeParams {
    params.is = scale_is(params.is, temperature, XTI_DIODE);
    params.temperature = temperature;
    params
}
