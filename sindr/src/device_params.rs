//! Effective device parameters for a [`CircuitElement`](crate::CircuitElement)'s fields.
//!
//! Every analysis that evaluates a nonlinear device (DC stamping, voltage
//! limiting, AC linearisation, transient stamping, result reporting) builds its
//! parameters here, so they all see the same temperature-scaled model as the
//! operating point that was solved.

use crate::circuit::{BjtModel, DiodeModel};
use sindr_devices::bjt::BjtParams;
use sindr_devices::diode::{
    temperature_scale_is, Breakdown, DiodeParams, JunctionCharge, K_OVER_Q, T_NOM,
};
use sindr_devices::led::{led_params_from_str, LedColor};
use sindr_devices::photodiode::PhotodiodeParams;
use sindr_devices::schottky::SchottkyParams;
use sindr_devices::zener::ZenerParams;

/// Silicon bandgap (eV).
const EG_SI: f64 = 1.11;

/// IS temperature exponent: SPICE's default XTI, for diodes and BJTs alike.
const XTI: f64 = 3.0;

/// Forward-voltage temperature coefficient (V/K) of an LED at its rated
/// 20 mA: the typical datasheet −2 mV/K, the same for every colour.
const LED_VF_TEMPCO: f64 = -2e-3;

/// `is` of a silicon junction with emission coefficient `n` (1 for a BJT),
/// given at nominal temperature, scaled to `temperature`.
fn scale_is(is: f64, temperature: f64, n: f64) -> f64 {
    scale_is_eg(is, temperature, n, EG_SI)
}

/// [`scale_is`] for a junction whose bandgap is `eg` (eV).
fn scale_is_eg(is: f64, temperature: f64, n: f64, eg: f64) -> f64 {
    if (temperature - T_NOM).abs() > 1e-6 {
        temperature_scale_is(is, temperature, T_NOM, eg, XTI, n)
    } else {
        is
    }
}

/// The EG (eV) an LED's IS scales with, fitted so its forward voltage `vf`
/// falls [`LED_VF_TEMPCO`] at 27 °C.
///
/// With SPICE's IS(T), a diode's `dVf/dT = (Vf − EG)/T − XTI·k/q` at fixed
/// current. Silicon's 1.11 eV, below every LED's Vf, made a hot LED's Vf
/// rise. The photon energy (≈ 1.9 eV for red) only gets it to about
/// −0.6 mV/K, so this is an effective value, a little above it.
fn led_eg(vf: f64) -> f64 {
    vf - T_NOM * (LED_VF_TEMPCO + XTI * K_OVER_Q)
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
            breakdown: model.bv.map(|bv| Breakdown { bv, ibv: model.ibv }),
        },
        temperature,
    )
}

/// The charge a diode with `model` stores, or `None` if it stores none.
pub(crate) fn diode_charge(model: Option<&DiodeModel>) -> Option<JunctionCharge> {
    let model = model.filter(|m| m.stores_charge())?;
    Some(JunctionCharge {
        cjo: model.cjo,
        vj: model.vj,
        m: model.m,
        fc: model.fc,
        tt: model.tt,
    })
}

/// LED of colour `color` at `temperature` (K). IS scales with the colour's
/// effective EG ([`led_eg`]), so Vf falls with temperature as a real LED's
/// does.
pub(crate) fn led(color: &str, temperature: f64) -> DiodeParams {
    let mut params = led_params_from_str(color);
    let eg = led_eg(LedColor::from_str(color).forward_voltage());
    params.is = scale_is_eg(params.is, temperature, params.n, eg);
    params.temperature = temperature;
    params
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
            breakdown: None,
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
        breakdown: None,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// An LED's Vf at its rated 20 mA falls about 2 mV/K, for every colour.
    #[test]
    fn hot_led_forward_voltage_falls() {
        for color in ["red", "green", "blue", "yellow", "white"] {
            let vf = |t: f64| {
                let p = led(color, t);
                p.n * p.vt() * (0.020 / p.is).ln_1p()
            };
            let tempco = (vf(T_NOM + 1.0) - vf(T_NOM - 1.0)) / 2.0;
            assert!(
                (-2.5e-3..=-1.5e-3).contains(&tempco),
                "{color}: {tempco} V/K"
            );
            // 27 °C is untouched.
            let rated = LedColor::from_str(color).forward_voltage();
            assert!((vf(T_NOM) - rated).abs() < 1e-9, "{color}");
        }
    }
}
