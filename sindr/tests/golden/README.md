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

Diode and BJT cases are SPICE decks that both simulators read, `.model` card
included. sindr reads `IS`, `N`, `RS` from a `D` card and `IS`, `BF`, `BR`,
`NF`, `NR`, `VAF` from an `NPN`/`PNP` card, and rejects any other parameter
(the goldens parse in strict mode), so a card here can't quietly lose one.
`dc_diode_1n4148` and `op_bjt_card` use non-default values. MOSFET, JFET,
LED, Schottky and photodiode have no SPICE card in sindr, so those tests
build the `Circuit` in Rust, element for element, and the `.cir` is read
only by ngspice.

### Decks written by `to_netlist`

`op_exported_controlled` and `ac_exported_transformer` build their circuit in
Rust, and their `.cir` is what `sindr::spice::to_netlist` writes for it. The
test checks that the committed `.cir` still matches, so the golden is
ngspice's answer for the very deck sindr exports. They pin the signs of E, G,
F, H and I sources and the transformer's dot convention. After changing the
exporter or one of these circuits:

```bash
SINDR_WRITE_DECKS=1 cargo test -p sindr --all-features --test golden exported
sindr/tests/golden/regen.sh op_exported_controlled ac_exported_transformer
```

## Adding a case

1. Write `<case>.cir` (and `.include` files next to it) plus `<case>.ctl`.
2. Run `regen.sh <case>`.
3. Add a `#[test]` in `golden.rs` that runs the same analysis on the sindr
   side and compares with `Check`, which reports the worst deviation of each
   quantity.

## Tolerances

- **Linear DC and AC:** 1e-6 relative.
- **Operating points, DC sweeps and junction devices:** 1e-6 relative on
  voltages and 1e-5 on currents. Cases with junctions run ngspice with
  `reltol=1e-9`: at its default 1e-3, ngspice itself stops a few µV short of
  the exact operating point.
- **Transient:** 0.1 % of full scale. sindr integrates with its default
  Gear-2 at a fixed step (dt = τ/50 over 5τ). ngspice can't reproduce a
  fixed step, so the golden is an accurate solution (`reltol=1e-6`, 1 µs
  max step) resampled onto sindr's grid with `linearize`. That leaves
  sindr's truncation error, about 0.03 % of full scale at the start, where
  the first step is backward Euler.

## Coverage

| Analysis | Cases |
|---|---|
| Operating point | `op_resistive`, `op_diode`, `op_diode_series`, `op_bjt_ce`, `op_bjt_card`, `op_exported_controlled` (E, G, F, H, I, potentiometer, switch), `op_mosfet_cs`, `op_mosfet_follower`, `op_pmos_highside`, `op_jfet`, `op_led`, `op_schottky`, `op_photodiode`, `op_subckt_param` (`.param`, nested `.subckt`, `.include`) |
| DC sweep | `dc_diode_iv`, `dc_diode_1n4148` |
| AC | `ac_rc_lowpass`, `ac_rlc_bandpass`, `ac_bjt_ce`, `ac_exported_transformer` |
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
- **`.lib`, `.nodeset`, `.ic`:** covered by `tests/spice_end_to_end.rs` and
  `tests/initial_conditions.rs`. A `.nodeset` doesn't change the answer, and
  sindr's `.ic` is always `UIC`, which ngspice runs differently.
