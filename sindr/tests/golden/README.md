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
`dc_diode_1n4148` and `op_bjt_card` use non-default values. The MOSFET
and JFET decks are read by sindr's parser too (`op_fet_decks_parse`,
`ac_fet_decks_parse`, `ac_mosfet_card`, `ac_jfet_caps`, `op_pjf`, and the vendor-style
subcircuit `dc_power_mosfet_subckt` / `dc_power_mosfet_body`, whose
`.model` cards sit inside the `.subckt`). LED, Schottky and photodiode
have no SPICE card in sindr, so those tests, and the Rust-built twins of
the FET decks, build the `Circuit` in Rust, element for element, and the
`.cir` is read only by ngspice. Each one passes its circuit through `in_deck`
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

### A deck driven from outside

`tran_session_square` is an RC driven by a `PULSE` square in the deck. sindr
drops the waveform and runs a `TransientSession` instead, setting the
source to each level at each millisecond as firmware would set a pin, and
checks that against ngspice's run of the pulse.

### Decks written by `to_netlist`

`op_exported_controlled` and `ac_exported_transformer` build their circuit in
Rust, and their `.cir` is what `sindr::spice::to_netlist` writes for it. The
test checks that the committed `.cir` still matches, so the golden is
ngspice's answer for the very deck sindr exports. They pin the signs of E, G,
F, H and I sources and the transformer's dot convention. Each test also
reads the committed deck back with sindr's own parser and checks that
against the same golden, so the E, G, F, H and K lines round-trip. The
composites (`op_exported_spdt`, `ac_exported_crystal`,
`ac_exported_ferrite_bead`, `dc_exported_digital_pin`) are checked the same
way, through the parts they export as. After changing the exporter or one of
these circuits:

```bash
SINDR_WRITE_DECKS=1 cargo test -p sindr --all-features --test golden exported
sindr/tests/golden/regen.sh op_exported_controlled ac_exported_transformer
```

### Sensitivity decks

The `sens_*` decks carry their `.sens` card, which sindr reads; the `.ctl`
says what ngspice does instead. ngspice 47's own `.sens` is only trusted
where it holds up:

- **Elements and controlled sources** (`sens_dc_controlled`,
  `sens_ac_linear`) come from ngspice's `.sens` (`r1`, `e1_gain`,
  `v1_acmag`, ...).
- **Device-model parameters** (`sens_dc_diode_mos`, `sens_dc_bjt`) come
  from central differences of re-solved ngspice operating points (an
  `altermod` 0.1 % either side, at `reltol=1e-10`), as `<id>_<param>`.
  ngspice's `.sens` reports 0 for a diode's RS and a BJT's BF, BR, NF and
  NR, is 2 % off for a diode's IS and N, and about 6e-4 off for a MOSFET's
  KP and VTO, against its own differences. No VAF: ngspice's Gummel–Poon
  Early effect isn't sindr's.
- **AC** at one frequency of a linear deck: ngspice's AC `.sens` gets only
  a sweep's first point right, and linearises at a zero bias, so a
  transistor would be off.

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
- **Fourier (`.four`):** the golden is ngspice's waveform; ngspice's
  `fourier` table, printed when the `.ctl` runs, is copied into the test.
  The deck carries a `.four 1k v(out)` card, and the test runs the
  `Fourier` it parses to, which must equal `Fourier::new(1e3)`.
  `Fourier` on ngspice's own waveform matches that table to 1e-5 relative
  in magnitude and 1e-3° in phase, which pins the algorithm. On sindr's
  waveform each harmonic is held to 0.1 % of the fundamental and the THD
  to 0.05 points (sindr reads 29.067 % against ngspice's 29.034 %).
- **Sensitivity:** 1e-6 relative for elements and controlled sources,
  DC and AC (each part of an AC phasor to 1e-6 of its magnitude); 1e-5 for
  device-model parameters against ngspice's central differences.
- **AC phase:** ngspice's `cph` unwraps the phase, so sindr's is compared
  a whole number of turns from it, to the magnitude's relative tolerance in
  radians (at least 1e-4°).

## Coverage

| Analysis | Cases |
|---|---|
| Operating point | `op_resistive`, `op_diode`, `op_diode_series`, `op_bjt_ce`, `op_bjt_pnp`, `op_bjt_card`, `op_exported_controlled` (E, G, F, H, I, potentiometer, switch), `op_exported_spdt`, `op_mosfet_cs`, `op_mosfet_follower`, `op_pmos_highside`, `op_jfet`, `op_led`, `op_schottky`, `op_photodiode`, `op_subckt_param` (`.param`, nested `.subckt`, `.include`) |
| DC sweep | `dc_diode_iv`, `dc_diode_1n4148`, `dc_diode_breakdown` (BV, IBV), `dc_zener` (a `ZenerDiode` through its knee), `dc_mosfet_body_diode` (an NMOS's body diode, freewheeling), `dc_exported_digital_pin` (push-pull, input with pull-up and open-drain pins pulled past both rails: the IBIS output stage and clamps they export as) |
| Diode charge | `cv_1n4148`, `cv_1n5819`: sindr's AC admittance at each bias of an ngspice sweep, against ngspice's `gd` and `cd` behind the card's RS |
| AC | `ac_rc_lowpass`, `ac_partial_decade` (`.ac dec` over 1.7 decades: ngspice's 17-point grid), `ac_rlc_bandpass`, `ac_bjt_ce`, `ac_bjt_caps` (Cbe, Cbc), `ac_diode` (1N4148 with its charge), `ac_mosfet_cs` (Cgs, Cgd), `ac_jfet`, `ac_jfet_caps` (CGS, CGD as graded junctions, from the `NJF` card), `ac_exported_transformer`, `ac_exported_crystal` (through both resonances), `ac_exported_ferrite_bead` (past its self-resonance) |
| Transient | `tran_rc_step`, `tran_rl_step`, `tran_op_start` (operating-point start), `tran_ic_op` and `tran_ic_uic` (`.ic` without and with `UIC`), `tran_tstart`, `tran_halfwave` (sine into a diode and filter cap), `tran_bjt_switch` (pulse through an RC into an NPN; collector to 3 %, see below), `tran_rlc_ring`, `tran_flasher` (a two-transistor LED flasher from rest: its period and the on LED's current, since each edge is a jump a fraction of a millisecond either way), `tran_diode_recovery` (1N4148 reverse recovery, 2 % of full scale: the recovery edge is steep, and sindr's adaptive step lands it a fraction of a nanosecond off ngspice's) |
| Fourier | `four_diode_clipper` (2 V sine through 1 kΩ into antiparallel diodes: odd harmonics to the 9th and a THD of 29 %) |
| Noise | `noise_rc_lowpass`, `noise_rc_total`, `noise_bjt_ce`, `noise_partial_decade` (`.noise dec` over 1.7 decades, which stops short of fstop) |
| Pole-zero | `pz_rlc_vol` (`.pz … vol pz`: a complex pole pair and a complex zero pair), `pz_ladder_cur` (`.pz … cur pz`: three poles and a zero at DC). ngspice writes nine significant digits, so roots are held to 1e-8 relative, and to 1e-8 of the largest root absolute (sindr's zero at DC comes out at round-off) |
| Temperature | `temp_diode` |
| Sensitivity | `sens_dc_controlled` (E, G, F, H gains), `sens_dc_diode_mos` (diode IS, N, RS; MOSFET KP, VTO, LAMBDA), `sens_dc_bjt` (BF, IS, BR, NF, NR), `sens_ac_linear` (R, C, L, E, G, F, H, AC magnitude at 1 kHz) |

None is ignored. `temp_diode` was, until the diode temperature fix; it reads
its deck and runs like the rest.

Not covered, and why:

- **IGBT:** ngspice has no matching primitive.
- **Varactor:** sindr's is a pure voltage-dependent capacitor. A SPICE diode
  with `CJO` also conducts.
- **Thermistor, photoresistor, potentiometer:** resistors whose value comes
  from a closed form, which unit tests cover; ngspice would only add a resistor.
- **`.lib`, `.nodeset`:** covered by `tests/spice_end_to_end.rs` and
  `tests/initial_conditions.rs`. A `.nodeset` doesn't change the answer.
