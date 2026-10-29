# Golden results from ngspice

`sindr/tests/golden.rs` checks sindr against reference results from
[ngspice](https://ngspice.sourceforge.io/). ngspice only produces the
reference data: sindr doesn't depend on it, and neither do the tests or CI.

## Regenerating

```bash
sindr/tests/golden/regen.sh            # every case
sindr/tests/golden/regen.sh op_diode   # one case
```

It needs `ngspice` on the PATH (macOS: `brew install ngspice`; Debian/Ubuntu:
`apt install ngspice`). The goldens were made with ngspice 47. Commit the
regenerated `.golden` files.

## A case

| File | What it is |
|---|---|
| `<case>.cir` | The netlist. Both simulators read it, unless the case is built in Rust (below). |
| `<case>.ctl` | ngspice commands for the analysis, ending in `wrdata @OUT@ <vectors>`. |
| `<case>.golden` | ngspice's `wrdata` table: a header of vector names, then rows; column 0 is the scale. |

`regen.sh` splices the `.ctl` into a `.control` block (sindr's parser doesn't
accept `.control`) and runs ngspice in batch mode at its default 27 °C.

`.model` cards spell out the parameters sindr's built-in models use, because
sindr's SPICE builder ignores most of them (diodes always use `IS=1e-14 N=1`,
BJTs honour only `BF`). MOSFET, JFET, LED, Schottky and photodiode have no
SPICE card in sindr, so those tests build the `Circuit` in Rust, element for
element, and the `.cir` is read only by ngspice.

## Adding a case

1. Write `<case>.cir` (and `.include` files next to it) plus `<case>.ctl`.
2. Run `regen.sh <case>`.
3. Add a `#[test]` in `golden.rs` that runs the same analysis on the sindr
   side and compares with `Check`, which reports the worst deviation of each
   quantity.

## Tolerances

- **Linear DC and AC:** 1e-6 relative.
- **Junction devices:** 1e-3 relative on voltages and 2e-2 on currents. sindr's
  thermal voltage is a rounded constant for 300.00 K, while devices and the
  goldens sit at 300.15 K. Re-running ngspice at sindr's V_T brings these
  cases under 2e-6. Tighten once SIN-15 lands.
- **Transient:** 1 % of full scale. sindr integrates with fixed-step backward
  Euler (dt = τ/50 over 5τ). ngspice can't reproduce a fixed step, so the
  golden is an accurate solution (`reltol=1e-6`, 1 µs max step) resampled
  onto sindr's grid with `linearize`. That leaves BE's truncation error,
  which peaks near 0.4 % of the step height.

## Coverage

| Analysis | Cases |
|---|---|
| Operating point | `op_resistive`, `op_diode`, `op_diode_series`, `op_bjt_ce`, `op_mosfet_cs`, `op_jfet`, `op_led`, `op_schottky`, `op_photodiode`, `op_subckt_param` (`.param`, nested `.subckt`, `.include`) |
| DC sweep | `dc_diode_iv` |
| AC | `ac_rc_lowpass`, `ac_rlc_bandpass`, `ac_bjt_ce` |
| Transient | `tran_rc_step`, `tran_rl_step` |
| Temperature | `temp_diode` |

Ignored until its bug is fixed: `temp_diode` (SIN-15, temperature maths).

Not covered, and why:

- **IGBT:** ngspice has no matching primitive.
- **Zener:** sindr's reverse breakdown is a piecewise `1/rz` clamp, which isn't
  SPICE's `BV`/`IBV` exponential. A comparison would only test the forward
  diode, which `op_diode` already does.
- **Varactor:** sindr's is a pure voltage-dependent capacitor. A SPICE diode
  with `CJO` also conducts.
- **Thermistor, photoresistor, potentiometer:** resistors whose value comes
  from a closed form, which unit tests cover; ngspice would only add a resistor.
- **`.lib`, `.nodeset`, `.ic`:** sindr's parser doesn't read `.nodeset` or
  `.ic`, and `.lib` is covered by `tests/spice_end_to_end.rs`.
