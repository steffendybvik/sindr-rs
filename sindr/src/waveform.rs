//! Time-varying waveform definitions for voltage/current sources.
//!
//! Each waveform variant implements `evaluate(t)` returning the instantaneous
//! value at time `t`. When attached to a VoltageSource or CurrentSource via
//! `waveform: Some(w)`, the source value at each transient timestep becomes
//! `dc_offset + w.evaluate(t)` (where dc_offset is the existing `voltage`/`current` field).

use std::f64::consts::PI;

/// A time-varying waveform.
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", serde(tag = "type"))]
pub enum Waveform {
    /// Sinusoidal: amplitude * sin(2*pi*frequency*t + phase) + offset
    #[cfg_attr(feature = "serde", serde(rename = "sine"))]
    Sine {
        amplitude: f64,
        frequency: f64,
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
        #[cfg_attr(feature = "serde", serde(default))]
        phase: f64, // radians
    },

    /// SPICE-compatible pulse waveform.
    #[cfg_attr(feature = "serde", serde(rename = "pulse"))]
    Pulse {
        v1: f64, // initial value
        v2: f64, // pulsed value
        #[cfg_attr(feature = "serde", serde(default))]
        delay: f64, // delay before first pulse (s)
        rise_time: f64, // rise time (s)
        fall_time: f64, // fall time (s)
        pulse_width: f64, // pulse width (s)
        period: f64, // period (s)
    },

    /// Square wave with configurable duty cycle.
    #[cfg_attr(feature = "serde", serde(rename = "square"))]
    Square {
        amplitude: f64,
        frequency: f64,
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
        #[cfg_attr(feature = "serde", serde(default = "default_duty"))]
        duty: f64, // 0.0 to 1.0, default 0.5
    },

    /// Triangle wave.
    #[cfg_attr(feature = "serde", serde(rename = "triangle"))]
    Triangle {
        amplitude: f64,
        frequency: f64,
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
    },

    /// PWM (pulse width modulation) — square wave with variable duty.
    #[cfg_attr(feature = "serde", serde(rename = "pwm"))]
    Pwm {
        amplitude: f64,
        frequency: f64,
        duty: f64, // 0.0 to 1.0
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
    },
}

fn default_duty() -> f64 {
    0.5
}

impl Waveform {
    /// Evaluate the waveform at time `t` (seconds).
    pub fn evaluate(&self, t: f64) -> f64 {
        match self {
            Waveform::Sine {
                amplitude,
                frequency,
                offset,
                phase,
            } => offset + amplitude * (2.0 * PI * frequency * t + phase).sin(),

            Waveform::Pulse {
                v1,
                v2,
                delay,
                rise_time,
                fall_time,
                pulse_width,
                period,
            } => {
                if t < *delay {
                    return *v1;
                }
                let t_rel = (t - delay) % period;
                if t_rel < *rise_time {
                    // Rising edge
                    let frac = if *rise_time > 0.0 {
                        t_rel / rise_time
                    } else {
                        1.0
                    };
                    v1 + (v2 - v1) * frac
                } else if t_rel < rise_time + pulse_width {
                    // Pulse high
                    *v2
                } else if t_rel < rise_time + pulse_width + fall_time {
                    // Falling edge
                    let frac = if *fall_time > 0.0 {
                        (t_rel - rise_time - pulse_width) / fall_time
                    } else {
                        1.0
                    };
                    v2 + (v1 - v2) * frac
                } else {
                    // Pulse low
                    *v1
                }
            }

            Waveform::Square {
                amplitude,
                frequency,
                offset,
                duty,
            } => {
                let t_rel = (t * frequency) % 1.0;
                if t_rel < *duty {
                    offset + amplitude
                } else {
                    offset - amplitude
                }
            }

            Waveform::Triangle {
                amplitude,
                frequency,
                offset,
            } => {
                let t_rel = (t * frequency) % 1.0;
                // Rising from -amp to +amp in first half, falling in second half
                let v = if t_rel < 0.5 {
                    -1.0 + 4.0 * t_rel
                } else {
                    3.0 - 4.0 * t_rel
                };
                offset + amplitude * v
            }

            Waveform::Pwm {
                amplitude,
                frequency,
                duty,
                offset,
            } => {
                let t_rel = (t * frequency) % 1.0;
                if t_rel < *duty {
                    offset + amplitude
                } else {
                    *offset
                }
            }
        }
    }

    /// Return the period of this waveform in seconds, if periodic.
    pub fn period(&self) -> Option<f64> {
        match self {
            Waveform::Sine { frequency, .. } => {
                if *frequency > 0.0 {
                    Some(1.0 / frequency)
                } else {
                    None
                }
            }
            Waveform::Pulse { period, .. } => Some(*period),
            Waveform::Square { frequency, .. }
            | Waveform::Triangle { frequency, .. }
            | Waveform::Pwm { frequency, .. } => {
                if *frequency > 0.0 {
                    Some(1.0 / frequency)
                } else {
                    None
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn sine_at_zero() {
        let w = Waveform::Sine {
            amplitude: 5.0,
            frequency: 1000.0,
            offset: 0.0,
            phase: 0.0,
        };
        assert_relative_eq!(w.evaluate(0.0), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn sine_at_quarter_period() {
        let w = Waveform::Sine {
            amplitude: 5.0,
            frequency: 1000.0,
            offset: 0.0,
            phase: 0.0,
        };
        // At t = 1/(4*f) = 0.25ms, sin(pi/2) = 1.0
        assert_relative_eq!(w.evaluate(0.25e-3), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn sine_with_offset_and_phase() {
        let w = Waveform::Sine {
            amplitude: 3.0,
            frequency: 100.0,
            offset: 2.0,
            phase: PI / 2.0,
        };
        // At t=0, sin(pi/2) = 1.0 => 2.0 + 3.0 = 5.0
        assert_relative_eq!(w.evaluate(0.0), 5.0, epsilon = 1e-12);
    }

    #[test]
    fn pulse_basic() {
        let w = Waveform::Pulse {
            v1: 0.0,
            v2: 5.0,
            delay: 0.0,
            rise_time: 1e-6,
            fall_time: 1e-6,
            pulse_width: 0.5e-3,
            period: 1e-3,
        };
        // During pulse high (after rise, before fall)
        assert_relative_eq!(w.evaluate(0.1e-3), 5.0, epsilon = 1e-10);
        // During pulse low
        assert_relative_eq!(w.evaluate(0.8e-3), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn pulse_with_delay() {
        let w = Waveform::Pulse {
            v1: 0.0,
            v2: 5.0,
            delay: 1e-3,
            rise_time: 0.0,
            fall_time: 0.0,
            pulse_width: 0.5e-3,
            period: 1e-3,
        };
        // Before delay
        assert_relative_eq!(w.evaluate(0.5e-3), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn square_wave() {
        let w = Waveform::Square {
            amplitude: 5.0,
            frequency: 1000.0,
            offset: 0.0,
            duty: 0.5,
        };
        // First half: +5V
        assert_relative_eq!(w.evaluate(0.1e-3), 5.0, epsilon = 1e-10);
        // Second half: -5V
        assert_relative_eq!(w.evaluate(0.6e-3), -5.0, epsilon = 1e-10);
    }

    #[test]
    fn triangle_wave() {
        let w = Waveform::Triangle {
            amplitude: 5.0,
            frequency: 1000.0,
            offset: 0.0,
        };
        // At t=0: start at -amplitude
        assert_relative_eq!(w.evaluate(0.0), -5.0, epsilon = 1e-10);
        // At t=T/4: midway up, at 0
        assert_relative_eq!(w.evaluate(0.25e-3), 0.0, epsilon = 1e-10);
        // At t=T/2: peak at +amplitude
        assert_relative_eq!(w.evaluate(0.5e-3), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn pwm_wave() {
        let w = Waveform::Pwm {
            amplitude: 3.3,
            frequency: 1000.0,
            duty: 0.25,
            offset: 0.0,
        };
        // First 25%: high (3.3V)
        assert_relative_eq!(w.evaluate(0.1e-3), 3.3, epsilon = 1e-10);
        // Remaining 75%: low (0V)
        assert_relative_eq!(w.evaluate(0.5e-3), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn period_calculation() {
        let sine = Waveform::Sine {
            amplitude: 1.0,
            frequency: 1000.0,
            offset: 0.0,
            phase: 0.0,
        };
        assert_relative_eq!(sine.period().unwrap(), 1e-3, epsilon = 1e-15);

        let pulse = Waveform::Pulse {
            v1: 0.0,
            v2: 5.0,
            delay: 0.0,
            rise_time: 0.0,
            fall_time: 0.0,
            pulse_width: 0.5e-3,
            period: 2e-3,
        };
        assert_relative_eq!(pulse.period().unwrap(), 2e-3, epsilon = 1e-15);
    }
}
