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
included. sindr reads `IS`, `N`, `RS`, `CJO`, `VJ`, `M`, `FC`, `TT`, `BV`, `IBV` from a `D` card and `IS`, `BF`, `BR`,
`NF`, `NR`, `VAF` from an `NPN`/`PNP` card, and rejects any other parameter
(the goldens parse in strict mode), so a card here can't quietly lose one.
`dc_diode_1n4148` and `op_bjt_card` use non-default values. MOSFET, JFET,
LED, Schottky and photodiode have no SPICE card in sindr, so those tests
build the `Circuit` in Rust, element for element, and the `.cir` is read
only by ngspice. Each one passes its circuit through `in_deck`
(`tests/support/deck_parity.rs`), which reads the `.cir` and fails unless
every element matches: names, nodes, values, and each `.model` parameter
against the Rust parameter it stands for (`KP·W/L` against `kp`,
`BETA·VTO²` against `idss`, LED and Schottky IS and N against sindr's
built-in values, a photodiode's current source against
responsivity × irradiance). A parameter the card leaves out is compared at
its SPICE default. A new Rust-built case must use it too.

### Decks with their own `.tran`

`tran_ic_op`, `tran_ic_uic` and `tran_tstart` keep the `.tran` and `.ic`
in the `.cir`, and their `.ctl` just says `run`. sindr runs the same cards
through `ParsedNetlist::transient_config()`, so these check the deck's
semantics (operating point or `UIC`, `.ic`, `tstart`) and not only the
solver.

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
   quantity. A circuit built in Rust goes through `in_deck` (or `exported`)
   first, so it can't drift from the deck.

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
- **Nonlinear transient** (`tran_halfwave`, `tran_bjt_switch`,
  `tran_rlc_ring`): the deck's own `.tran` with `uic`, so both start from
  rest, and a short TMAX in the deck, so the check is the physics and not
  the step control: 0.1 % of full scale. At default steps sindr is off by
  0.2 % to 3 % of full scale on the rectifier's charging peaks and the
  ringing, a little more than ngspice at its own defaults (0.05 % to 2 %).
  `tran_bjt_switch`'s collector is checked to 3 %: in the active region
  ic = IS·exp(vbe/Vt) turns the base's 0.4 mV error into about 2.7 % of ic
  on an edge that falls 5 V in 3 µs. The base itself is held to 0.1 %.
- **AC phase:** ngspice's `cph` unwraps the phase, so sindr's is compared
  a whole number of turns from it, to the magnitude's relative tolerance in
  radians (at least 1e-4°).

## Coverage

| Analysis | Cases |
|---|---|
| Operating point | `op_resistive`, `op_diode`, `op_diode_series`, `op_bjt_ce`, `op_bjt_card`, `op_exported_controlled` (E, G, F, H, I, potentiometer, switch), `op_exported_spdt`, `op_mosfet_cs`, `op_mosfet_follower`, `op_pmos_highside`, `op_jfet`, `op_led`, `op_schottky`, `op_photodiode`, `op_subckt_param` (`.param`, nested `.subckt`, `.include`) |
| DC sweep | `dc_diode_iv`, `dc_diode_1n4148`, `dc_diode_breakdown` (BV, IBV), `dc_zener` (a `ZenerDiode` through its knee), `dc_mosfet_body_diode` (an NMOS's body diode, freewheeling) |
| Diode charge | `cv_1n4148`, `cv_1n5819`: sindr's AC admittance at each bias of an ngspice sweep, against ngspice's `gd` and `cd` behind the card's RS |
| AC | `ac_rc_lowpass`, `ac_rlc_bandpass`, `ac_bjt_ce`, `ac_bjt_caps` (Cbe, Cbc), `ac_diode` (1N4148 with its charge), `ac_mosfet_cs` (Cgs, Cgd), `ac_jfet`, `ac_exported_transformer`, `ac_exported_crystal` (through both resonances), `ac_exported_ferrite_bead` (past its self-resonance) |
| Transient | `tran_rc_step`, `tran_rl_step`, `tran_op_start` (operating-point start), `tran_ic_op` and `tran_ic_uic` (`.ic` without and with `UIC`), `tran_tstart`, `tran_halfwave` (sine into a diode and filter cap), `tran_bjt_switch` (pulse through an RC into an NPN; collector to 3 %, see below), `tran_rlc_ring`, `tran_diode_recovery` (1N4148 reverse recovery, 2 % of full scale: the recovery edge is steep, and sindr's adaptive step lands it a fraction of a nanosecond off ngspice's) |
| Temperature | `temp_diode` |

None is ignored. `temp_diode` was, until SIN-15's temperature fix; it reads
its deck and runs like the rest.

Not covered, and why:

- **IGBT:** ngspice has no matching primitive.
- **Varactor:** sindr's is a pure voltage-dependent capacitor. A SPICE diode
  with `CJO` also conducts.
- **Thermistor, photoresistor, potentiometer:** resistors whose value comes
  from a closed form, which unit tests cover; ngspice would only add a resistor.
- **`.lib`, `.nodeset`:** covered by `tests/spice_end_to_end.rs` and
  `tests/initial_conditions.rs`. A `.nodeset` doesn't change the answer.
