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
        /// Peak amplitude (V or A).
        amplitude: f64,
        /// Frequency (Hz).
        frequency: f64,
        /// DC offset added to the sine (V or A). Default 0.
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
        /// Phase offset (radians). Default 0.
        #[cfg_attr(feature = "serde", serde(default))]
        phase: f64, // radians
    },

    /// SPICE-compatible pulse waveform.
    #[cfg_attr(feature = "serde", serde(rename = "pulse"))]
    Pulse {
        /// Initial (low) value (V or A).
        v1: f64, // initial value
        /// Pulsed (high) value (V or A).
        v2: f64, // pulsed value
        /// Delay before the first pulse (s). Default 0.
        #[cfg_attr(feature = "serde", serde(default))]
        delay: f64, // delay before first pulse (s)
        /// Rise time from `v1` to `v2` (s).
        rise_time: f64, // rise time (s)
        /// Fall time from `v2` back to `v1` (s).
        fall_time: f64, // fall time (s)
        /// Width of the high portion of the pulse (s).
        pulse_width: f64, // pulse width (s)
        /// Repetition period (s).
        period: f64, // period (s)
    },

    /// Square wave with configurable duty cycle.
    #[cfg_attr(feature = "serde", serde(rename = "square"))]
    Square {
        /// Peak amplitude about the offset (V or A).
        amplitude: f64,
        /// Frequency (Hz).
        frequency: f64,
        /// DC offset added to the wave (V or A). Default 0.
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
        /// Duty cycle (fraction high), 0.0 to 1.0. Default 0.5.
        #[cfg_attr(feature = "serde", serde(default = "default_duty"))]
        duty: f64, // 0.0 to 1.0, default 0.5
    },

    /// Triangle wave.
    #[cfg_attr(feature = "serde", serde(rename = "triangle"))]
    Triangle {
        /// Peak amplitude about the offset (V or A).
        amplitude: f64,
        /// Frequency (Hz).
        frequency: f64,
        /// DC offset added to the wave (V or A). Default 0.
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
    },

    /// PWM (pulse width modulation) — square wave with variable duty.
    #[cfg_attr(feature = "serde", serde(rename = "pwm"))]
    Pwm {
        /// High-level amplitude above the offset (V or A).
        amplitude: f64,
        /// Frequency (Hz).
        frequency: f64,
        /// Duty cycle (fraction high), 0.0 to 1.0.
        duty: f64, // 0.0 to 1.0
        /// DC offset (low level) (V or A). Default 0.
        #[cfg_attr(feature = "serde", serde(default))]
        offset: f64,
    },

    /// Piecewise-linear waveform. `points` is a list of `(t, value)`
    /// breakpoints sorted by `t`. Values are linearly interpolated between
    /// adjacent points; for `t < points[0].0` the first value is held, and
    /// for `t > points.last().0` the last value is held. An empty list
    /// evaluates to `0.0` everywhere.
    #[cfg_attr(feature = "serde", serde(rename = "pwl"))]
    Pwl {
        /// Breakpoints as `(time_s, value)` pairs sorted by time.
        points: Vec<(f64, f64)>,
    },
}

/// Most breakpoints [`Waveform::breakpoints`] returns.
pub const MAX_BREAKPOINTS: usize = 10_000_000;

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

            Waveform::Pwl { points } => {
                if points.is_empty() {
                    return 0.0;
                }
                // Edge clamps: hold first value before first t, last value after last t.
                if t <= points[0].0 {
                    return points[0].1;
                }
                let last = points.len() - 1;
                if t >= points[last].0 {
                    return points[last].1;
                }
                // Binary search for the bracketing pair (t_i, t_{i+1}) with t_i <= t < t_{i+1}.
                let idx = match points.binary_search_by(|(ti, _)| {
                    ti.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal)
                }) {
                    Ok(i) => return points[i].1,
                    Err(i) => i, // first index where points[i].0 > t
                };
                let (t0, v0) = points[idx - 1];
                let (t1, v1) = points[idx];
                let dt = t1 - t0;
                if dt == 0.0 {
                    v0
                } else {
                    v0 + (v1 - v0) * ((t - t0) / dt)
                }
            }
        }
    }

    /// Times in `(0, t_end)` (s) where this waveform has a corner or a jump:
    /// pulse edges, square/PWM transitions, triangle peaks, PWL points. A sine
    /// has none. Sorted; at most [`MAX_BREAKPOINTS`] of them.
    ///
    /// The transient stepper lands a step exactly on each one, so a fast
    /// edge between two steps isn't stepped over.
    ///
    /// ```
    /// use sindr::Waveform;
    ///
    /// let pulse = Waveform::Pulse {
    ///     v1: 0.0, v2: 5.0, delay: 1e-3, rise_time: 1e-9, fall_time: 1e-9,
    ///     pulse_width: 2e-3, period: 10e-3,
    /// };
    /// assert_eq!(pulse.breakpoints(5e-3), vec![1e-3, 1e-3 + 1e-9, 3e-3 + 1e-9, 3e-3 + 2e-9]);
    /// ```
    pub fn breakpoints(&self, t_end: f64) -> Vec<f64> {
        let mut out = Vec::new();
        // Corners at `start + k·period + offset` for each offset, k ≥ 0.
        let mut periodic = |start: f64, period: f64, offsets: &[f64]| {
            if !(period > 0.0 && period.is_finite()) {
                out.extend(offsets.iter().map(|o| start + o));
                return;
            }
            let mut k = 0.0;
            while start + k * period < t_end && out.len() < MAX_BREAKPOINTS {
                out.extend(offsets.iter().map(|o| start + k * period + o));
                k += 1.0;
            }
        };
        match self {
            Waveform::Sine { .. } => {}
            Waveform::Pulse {
                delay,
                rise_time,
                fall_time,
                pulse_width,
                period,
                ..
            } => {
                let (tr, pw, tf) = (*rise_time, *pulse_width, *fall_time);
                periodic(*delay, *period, &[0.0, tr, tr + pw, tr + pw + tf]);
            }
            Waveform::Square {
                frequency, duty, ..
            }
            | Waveform::Pwm {
                frequency, duty, ..
            } => periodic(0.0, 1.0 / frequency, &[0.0, duty / frequency]),
            Waveform::Triangle { frequency, .. } => {
                periodic(0.0, 1.0 / frequency, &[0.0, 0.5 / frequency])
            }
            Waveform::Pwl { points } => out.extend(points.iter().map(|p| p.0)),
        }
        out.retain(|t| *t > 0.0 && *t < t_end && t.is_finite());
        out.sort_by(f64::total_cmp);
        out.dedup();
        out.truncate(MAX_BREAKPOINTS);
        out
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
            // PWL has no inherent period.
            Waveform::Pwl { .. } => None,
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn breakpoints_of_each_waveform() {
        let sine = Waveform::Sine {
            amplitude: 1.0,
            frequency: 1e3,
            offset: 0.0,
            phase: 0.0,
        };
        assert!(sine.breakpoints(1.0).is_empty());

        // Square at 1 kHz, 25 % duty: flips at k ms and k ms + 0.25 ms.
        let square = Waveform::Square {
            amplitude: 1.0,
            frequency: 1e3,
            offset: 0.0,
            duty: 0.25,
        };
        let got = square.breakpoints(2.1e-3);
        let want = [0.25e-3, 1e-3, 1.25e-3, 2e-3];
        assert_eq!(got.len(), want.len(), "{got:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-15, "{got:?}");
        }

        // Triangle peaks at half periods.
        let triangle = Waveform::Triangle {
            amplitude: 1.0,
            frequency: 1e3,
            offset: 0.0,
        };
        assert_eq!(triangle.breakpoints(1.6e-3).len(), 3);

        // PWL corners inside the window only.
        let pwl = Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 1.0), (2e-3, 1.0), (5e-3, 0.0)],
        };
        assert_eq!(pwl.breakpoints(3e-3), vec![1e-3, 2e-3]);
    }

    /// A pulse with a zero period is a single pulse, not an endless loop.
    #[test]
    fn pulse_with_zero_period_has_one_set_of_corners() {
        let pulse = Waveform::Pulse {
            v1: 0.0,
            v2: 1.0,
            delay: 1e-3,
            rise_time: 1e-6,
            fall_time: 1e-6,
            pulse_width: 1e-3,
            period: 0.0,
        };
        assert_eq!(pulse.breakpoints(1.0).len(), 4);
    }

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
    fn pwl_interpolates_between_breakpoints() {
        let w = Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 5.0)],
        };
        // Midpoint should linearly interpolate to 2.5
        assert_relative_eq!(w.evaluate(0.5e-3), 2.5, epsilon = 1e-12);
    }

    #[test]
    fn pwl_clamps_at_edges() {
        let w = Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 5.0)],
        };
        // Before first breakpoint: hold first value
        assert_relative_eq!(w.evaluate(-1.0), 0.0, epsilon = 1e-12);
        // After last breakpoint: hold last value
        assert_relative_eq!(w.evaluate(1.0), 5.0, epsilon = 1e-12);
        // Exactly at a breakpoint
        assert_relative_eq!(w.evaluate(0.0), 0.0, epsilon = 1e-12);
        assert_relative_eq!(w.evaluate(1e-3), 5.0, epsilon = 1e-12);
    }

    #[test]
    fn pwl_empty_returns_zero() {
        let w = Waveform::Pwl { points: vec![] };
        assert_relative_eq!(w.evaluate(0.0), 0.0, epsilon = 1e-12);
        assert_relative_eq!(w.evaluate(42.0), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn pwl_multi_segment() {
        // 0->1 then 1->-1 then -1->0
        let w = Waveform::Pwl {
            points: vec![(0.0, 0.0), (1.0, 1.0), (2.0, -1.0), (3.0, 0.0)],
        };
        assert_relative_eq!(w.evaluate(0.5), 0.5, epsilon = 1e-12);
        assert_relative_eq!(w.evaluate(1.5), 0.0, epsilon = 1e-12);
        assert_relative_eq!(w.evaluate(2.5), -0.5, epsilon = 1e-12);
    }

    #[test]
    fn pwl_period_is_none() {
        let w = Waveform::Pwl {
            points: vec![(0.0, 0.0), (1e-3, 5.0)],
        };
        assert!(w.period().is_none());
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
