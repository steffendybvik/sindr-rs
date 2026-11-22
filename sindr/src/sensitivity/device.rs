//! Device-model parameters for sensitivity analysis: how a diode's, BJT's or
//! MOSFET's terminal currents (DC) and small-signal admittances (AC) move
//! with each of its parameters, the bias held where it is.

use nalgebra::DVector;
use num_complex::Complex64;
use sindr_devices::bjt::{bjt_derivatives, BjtDerivative, BjtKind};
use sindr_devices::diode::{self, diode_derivatives, DiodeDerivative, GMIN};
use sindr_devices::mosfet::{mosfet_derivatives, MosfetDerivative, MosfetKind};

use super::SensParameter;
use crate::circuit::CircuitElement;
use crate::device_params;
use crate::node_map::NodeMap;

/// One device-model parameter and its derivative at the bias point.
pub(super) struct DeviceParameter<T> {
    pub(super) parameter: SensParameter,
    pub(super) value: f64,
    /// DC: ∂I/∂p of the current each node sends into the device. AC: ∂Y/∂p
    /// of each entry `(row, col)` of the admittance the device stamps.
    pub(super) derivative: Vec<T>,
}

/// A DC residual derivative: the node (`None` for ground) and ∂I/∂p (A per
/// unit of the parameter).
pub(super) type DcEntry = (Option<usize>, f64);

/// An AC matrix derivative: row, column, and ∂Y/∂p.
pub(super) type AcEntry = (Option<usize>, Option<usize>, Complex64);

fn voltage(node_map: &NodeMap, x: &DVector<f64>, node: &str) -> (Option<usize>, f64) {
    let i = node_map.index(node);
    (i, i.map_or(0.0, |k| x[k]))
}

/// How a diode's temperature-scaled IS moves with the model's IS and N:
/// `(∂IS(T)/∂IS, ∂IS(T)/∂N)`. IS(T) = IS·exp(E/N) for an `E` that holds
/// XTI, EG and T, so the second is −IS(T)·ln(IS(T)/IS)/N.
fn is_scaling(is: f64, is_t: f64, n: f64) -> (f64, f64) {
    if is > 0.0 && is_t > 0.0 {
        let ratio = is_t / is;
        (ratio, -is_t * ratio.ln() / n)
    } else {
        (1.0, 0.0)
    }
}

/// The diode's parameter derivatives with the temperature scaling of IS
/// folded in: `(parameter, value, derivative)` for IS, N and RS.
fn diode_parameters(
    model: &crate::DiodeModel,
    params: &diode::DiodeParams,
    v_d: f64,
) -> [(SensParameter, f64, DiodeDerivative); 3] {
    let d = diode_derivatives(v_d, params);
    let (dis_dis, dis_dn) = is_scaling(model.is, params.is, model.n);
    let scaled = |mut a: DiodeDerivative, k: f64| {
        a.current *= k;
        a.junction_voltage *= k;
        a.junction_conductance *= k;
        a
    };
    let sum = |mut a: DiodeDerivative, b: DiodeDerivative| {
        a.current += b.current;
        a.junction_voltage += b.junction_voltage;
        a.junction_conductance += b.junction_conductance;
        a
    };
    [
        (
            SensParameter::SaturationCurrent,
            model.is,
            scaled(d.is, dis_dis),
        ),
        (
            SensParameter::EmissionCoefficient,
            model.n,
            sum(d.n, scaled(d.is, dis_dn)),
        ),
        (SensParameter::SeriesResistance, model.rs, d.rs),
    ]
}

/// The BJT's parameters, with the derivative of each in the NPN frame: BF,
/// IS (through its temperature scaling), BR, NF, NR, and VAF when the Early
/// effect is on.
fn bjt_parameters(
    bf: f64,
    model: Option<&crate::BjtModel>,
    temperature: f64,
    vbe: f64,
    vbc: f64,
) -> Vec<(SensParameter, f64, BjtDerivative)> {
    let m = model.copied().unwrap_or_default();
    let params = device_params::bjt(bf, model, temperature);
    let d = bjt_derivatives(vbe, vbc, &params);
    // A BJT's IS scales with N = 1, so only IS itself moves IS(T).
    let (dis_dis, _) = is_scaling(m.is, params.is, 1.0);
    let scaled = |a: BjtDerivative, k: f64| {
        let mut b = a;
        b.ib *= k;
        b.ic *= k;
        b.jacobian.dib_dvbe *= k;
        b.jacobian.dib_dvbc *= k;
        b.jacobian.dic_dvbe *= k;
        b.jacobian.dic_dvbc *= k;
        b
    };
    let mut out = vec![
        (SensParameter::ForwardBeta, bf, d.bf),
        (
            SensParameter::SaturationCurrent,
            m.is,
            scaled(d.is, dis_dis),
        ),
        (SensParameter::ReverseBeta, m.br, d.br),
        (SensParameter::ForwardEmissionCoefficient, m.nf, d.nf),
        (SensParameter::ReverseEmissionCoefficient, m.nr, d.nr),
    ];
    if m.vaf > 0.0 && m.vaf.is_finite() {
        out.push((SensParameter::EarlyVoltage, m.vaf, d.vaf));
    }
    out
}

/// The MOSFET's KP, VTO and LAMBDA with their derivatives, VTO's taken
/// back from the N-channel frame to its SPICE sign.
fn mosfet_parameters(
    kind: MosfetKind,
    params: &sindr_devices::mosfet::MosfetParams,
    vg: f64,
    vd: f64,
    vs: f64,
) -> [(SensParameter, f64, MosfetDerivative); 3] {
    let (vgs, vds) = kind.nmos_frame(vg, vd, vs);
    let d = mosfet_derivatives(vgs, vds, 0.0, &params.in_nmos_frame(kind));
    let frame = polarity(kind);
    let mut vto = d.vto;
    vto.id *= frame;
    vto.gm *= frame;
    vto.gds *= frame;
    [
        (SensParameter::TransconductanceParameter, params.kp, d.kp),
        (SensParameter::ThresholdVoltage, params.vto, vto),
        (
            SensParameter::ChannelLengthModulation,
            params.lambda,
            d.lambda,
        ),
    ]
}

fn polarity(kind: MosfetKind) -> f64 {
    match kind {
        MosfetKind::Nmos => 1.0,
        MosfetKind::Pmos => -1.0,
    }
}

fn bjt_polarity(kind: BjtKind) -> f64 {
    match kind {
        BjtKind::Npn => 1.0,
        BjtKind::Pnp => -1.0,
    }
}

/// The device-model parameters of `component` and how the current each of
/// its nodes sends into it moves with them, at the DC solution `x`. Empty
/// for anything but a diode, BJT or MOSFET.
pub(super) fn dc_parameters(
    component: &CircuitElement,
    node_map: &NodeMap,
    x: &DVector<f64>,
) -> Vec<DeviceParameter<DcEntry>> {
    let v = |node: &str| voltage(node_map, x, node);
    match component {
        CircuitElement::Diode {
            nodes,
            temperature,
            model,
            ..
        } => {
            let ((p, vp), (q, vq)) = (v(&nodes[0]), v(&nodes[1]));
            let params = device_params::diode(model.as_ref(), *temperature);
            let m = model.unwrap_or_default();
            diode_parameters(&m, &params, vp - vq)
                .into_iter()
                .map(|(parameter, value, d)| DeviceParameter {
                    parameter,
                    value,
                    derivative: vec![(p, d.current), (q, -d.current)],
                })
                .collect()
        }
        CircuitElement::Bjt {
            nodes,
            kind,
            bf,
            temperature,
            model,
            ..
        } => {
            let ((b, vb), (c, vc), (e, ve)) = (v(&nodes[0]), v(&nodes[1]), v(&nodes[2]));
            let s = bjt_polarity(*kind);
            bjt_parameters(
                *bf,
                model.as_ref(),
                *temperature,
                s * (vb - ve),
                s * (vb - vc),
            )
            .into_iter()
            .map(|(parameter, value, d)| DeviceParameter {
                parameter,
                value,
                derivative: vec![(b, s * d.ib), (c, s * d.ic), (e, -s * (d.ib + d.ic))],
            })
            .collect()
        }
        CircuitElement::Mosfet {
            nodes,
            kind,
            params,
            ..
        } => {
            let ((_, vg), (d_node, vd), (s_node, vs)) = (v(&nodes[0]), v(&nodes[1]), v(&nodes[2]));
            let s = polarity(*kind);
            mosfet_parameters(*kind, params, vg, vd, vs)
                .into_iter()
                .map(|(parameter, value, d)| DeviceParameter {
                    parameter,
                    value,
                    derivative: vec![(d_node, s * d.id), (s_node, -s * d.id)],
                })
                .collect()
        }
        _ => Vec::new(),
    }
}

/// ∂Y/∂p of a conductance-like derivative `dy` between `p` and `q`.
fn two_terminal(p: Option<usize>, q: Option<usize>, dy: Complex64) -> Vec<AcEntry> {
    vec![(p, p, dy), (q, q, dy), (p, q, -dy), (q, p, -dy)]
}

/// The device-model parameters of `component` and how its small-signal
/// admittance at `omega` (rad/s) moves with them, the DC solution `x` held:
/// the bias isn't re-solved, as in SPICE's AC sensitivity. Empty for
/// anything but a diode, BJT or MOSFET.
pub(super) fn ac_parameters(
    component: &CircuitElement,
    node_map: &NodeMap,
    x: &DVector<f64>,
    omega: f64,
) -> Vec<DeviceParameter<AcEntry>> {
    let v = |node: &str| voltage(node_map, x, node);
    let j_omega = Complex64::new(0.0, omega);
    match component {
        CircuitElement::Diode {
            nodes,
            temperature,
            model,
            ..
        } => {
            let ((p, vp), (q, vq)) = (v(&nodes[0]), v(&nodes[1]));
            let v_d = vp - vq;
            let params = device_params::diode(model.as_ref(), *temperature);
            let m = model.unwrap_or_default();
            let charge = device_params::diode_charge(model.as_ref());
            let rs = params.rs;
            let i_d = diode::diode_current(v_d, &params);
            let v_j = v_d - rs * i_d;
            let g_j = diode::junction_current(v_j, &params).1;
            // The admittance AC stamps: g_j + jωC_j behind RS, or with no
            // stored charge the companion's g_d, whose GMIN floor is flat.
            let c_j = charge.map_or(0.0, |c| c.charge(v_j, i_d, g_j).1);
            let junction = Complex64::new(g_j, 0.0) + j_omega * c_j;
            let through_rs = Complex64::new(1.0, 0.0) + junction * rs;
            let floored = charge.is_none() && g_j / (1.0 + rs * g_j) < GMIN;
            // ∂Y/∂p for ∂J/∂p and ∂RS/∂p.
            let admittance = |dj: Complex64, d_rs: f64| {
                if floored || !through_rs.is_finite() {
                    return Complex64::new(0.0, 0.0);
                }
                (dj - junction * junction * d_rs) / (through_rs * through_rs)
            };
            let mut out: Vec<DeviceParameter<AcEntry>> = diode_parameters(&m, &params, v_d)
                .into_iter()
                .map(|(parameter, value, d)| {
                    let dc_j = charge.map_or(0.0, |c| {
                        c.tt * d.junction_conductance
                            + c.depletion_capacitance_slope(v_j) * d.junction_voltage
                    });
                    let dj = Complex64::new(d.junction_conductance, 0.0) + j_omega * dc_j;
                    let d_rs = f64::from(u8::from(parameter == SensParameter::SeriesResistance));
                    DeviceParameter {
                        parameter,
                        value,
                        derivative: two_terminal(p, q, admittance(dj, d_rs)),
                    }
                })
                .collect();
            if let Some(c) = charge {
                // C_j = TT·g_j + CJO·(the depletion shape at v_j).
                let unit = sindr_devices::diode::JunctionCharge {
                    cjo: 1.0,
                    tt: 0.0,
                    ..c
                };
                let shape = unit.charge(v_j, 0.0, 0.0).1;
                for (parameter, value, dc) in [
                    (SensParameter::JunctionCapacitance, c.cjo, shape),
                    (SensParameter::TransitTime, c.tt, g_j),
                ] {
                    out.push(DeviceParameter {
                        parameter,
                        value,
                        derivative: two_terminal(p, q, admittance(j_omega * dc, 0.0)),
                    });
                }
            }
            out
        }
        CircuitElement::Bjt {
            nodes,
            kind,
            bf,
            temperature,
            model,
            ..
        } => {
            let ((b, vb), (c, vc), (e, ve)) = (v(&nodes[0]), v(&nodes[1]), v(&nodes[2]));
            let s = bjt_polarity(*kind);
            let terminals = [b, c, e];
            bjt_parameters(
                *bf,
                model.as_ref(),
                *temperature,
                s * (vb - ve),
                s * (vb - vc),
            )
            .into_iter()
            .map(|(parameter, value, d)| {
                // The terminal Jacobian is the same for NPN and PNP, and
                // linear in its four entries.
                let m = d.jacobian.terminal_matrix();
                let mut derivative = Vec::with_capacity(9);
                for (r, &row) in terminals.iter().enumerate() {
                    for (k, &col) in terminals.iter().enumerate() {
                        derivative.push((row, col, Complex64::new(m[r][k], 0.0)));
                    }
                }
                DeviceParameter {
                    parameter,
                    value,
                    derivative,
                }
            })
            .collect()
        }
        CircuitElement::Mosfet {
            nodes,
            kind,
            params,
            ..
        } => {
            let ((g, vg), (d_node, vd), (s_node, vs)) = (v(&nodes[0]), v(&nodes[1]), v(&nodes[2]));
            mosfet_parameters(*kind, params, vg, vd, vs)
                .into_iter()
                .map(|(parameter, value, d)| {
                    let (gm, gds) = (Complex64::new(d.gm, 0.0), Complex64::new(d.gds, 0.0));
                    DeviceParameter {
                        parameter,
                        value,
                        derivative: vec![
                            (d_node, g, gm),
                            (d_node, s_node, -gm),
                            (s_node, g, -gm),
                            (s_node, s_node, gm),
                            (d_node, d_node, gds),
                            (s_node, s_node, gds),
                            (d_node, s_node, -gds),
                            (s_node, d_node, -gds),
                        ],
                    }
                })
                .collect()
        }
        _ => Vec::new(),
    }
}
