//! Effective device parameters for a [`CircuitElement`](crate::CircuitElement)'s fields.
//!
//! Every analysis that evaluates a nonlinear device (DC stamping, voltage
//! limiting, AC linearisation, transient stamping, result reporting) builds its
//! parameters here, so they all see the same temperature-scaled model as the
//! operating point that was solved.

use crate::circuit::{BjtModel, DiodeModel};
use sindr_devices::bjt::BjtParams;
use sindr_devices::diode::{temperature_scale_is, DiodeParams, T_NOM};
use sindr_devices::led::led_params_from_str;
use sindr_devices::photodiode::PhotodiodeParams;
use sindr_devices::schottky::SchottkyParams;
use sindr_devices::zener::ZenerParams;

/// Silicon bandgap (eV).
const EG_SI: f64 = 1.11;

/// IS temperature exponent: SPICE's default XTI, for diodes and BJTs alike.
const XTI: f64 = 3.0;

/// `is` of a junction with emission coefficient `n` (1 for a BJT), given at
/// nominal temperature, scaled to `temperature`.
fn scale_is(is: f64, temperature: f64, n: f64) -> f64 {
    if (temperature - T_NOM).abs() > 1e-6 {
        temperature_scale_is(is, temperature, T_NOM, EG_SI, XTI, n)
    } else {
        is
    }
}

/// Diode with `model` (the built-in silicon diode if `None`) at `temperature` (K).
pub(crate) fn diode(model: Option<&DiodeModel>, temperature: f64) -> DiodeParams {
    let model = model.copied().unwrap_or_default();
    with_temperature(
        DiodeParams {
            is: model.is,
            n: model.n,
            rs: model.rs,
            temperature: T_NOM,
        },
        temperature,
    )
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
    params.is = scale_is(params.is, temperature, 1.0);
    params.temperature = temperature;
    params
}

/// Photodiode at `temperature` (K). The dark current scales like a diode's;
/// responsivity is taken as temperature-independent.
pub(crate) fn photodiode(temperature: f64) -> PhotodiodeParams {
    let mut params = PhotodiodeParams::default();
    params.is = scale_is(params.is, temperature, params.n);
    params.temperature = temperature;
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

/// BJT with forward gain `bf` and `model` (the built-in transistor if
/// `None`) at `temperature` (K).
pub(crate) fn bjt(bf: f64, model: Option<&BjtModel>, temperature: f64) -> BjtParams {
    let model = model.copied().unwrap_or_default();
    let mut params = BjtParams::new(bf);
    params.br = model.br;
    params.nf = model.nf;
    params.nr = model.nr;
    params.vaf = model.vaf;
    params.is = scale_is(model.is, temperature, 1.0);
    params.temperature = temperature;
    params
}

fn with_temperature(mut params: DiodeParams, temperature: f64) -> DiodeParams {
    params.is = scale_is(params.is, temperature, params.n);
    params.temperature = temperature;
    params
}

/// Resistance of the built-in NTC thermistor at `temperature` (K), in Ω.
/// Every analysis and the SPICE exporter read it from here, so they agree.
pub(crate) fn thermistor_resistance(temperature: f64) -> f64 {
    let params = sindr_devices::thermistor::ThermistorParams::default();
    sindr_devices::thermistor::thermistor_resistance(temperature, &params)
}
