#!/usr/bin/env python3
"""Fail when a source file's line coverage drops below its floor.

    cargo llvm-cov --workspace --all-features --summary-only > coverage.txt
    python3 scripts/coverage_floor.py coverage.txt

Reads the `cargo llvm-cov` summary table. Each floor sits one or two points
under the coverage the file had when it was set, so noise doesn't fail a
build but a block of untested code does. Every source file has a floor, so
a renamed file fails until its floor is renamed too; a new file is covered
by TOTAL until it gets one. Raise a floor when a file's
coverage rises; lower one only with a reason in the PR.
"""

import sys

# Line coverage (%) each file must keep.
FLOORS = {
    "sindr-devices/src/battery.rs": 96.5,  # 98.06% when set
    "sindr-devices/src/bjt.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/converter.rs": 96.5,  # 98.49% when set
    "sindr-devices/src/datasheet.rs": 89.5,  # 91.43% when set
    "sindr-devices/src/diode.rs": 98.0,  # 99.79% when set
    "sindr-devices/src/igbt.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/inductor.rs": 97.0,  # 98.70% when set
    "sindr-devices/src/jfet.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/led.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/mosfet.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/photodiode.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/photoresistor.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/regulator.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/schottky.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/thermistor.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/varactor.rs": 98.5,  # 100.00% when set
    "sindr-devices/src/zener.rs": 98.5,  # 100.00% when set
    "sindr/src/ac_analysis.rs": 95.5,  # 97.37% when set
    "sindr/src/battery_state.rs": 98.5,  # 100.00% when set
    "sindr/src/circuit.rs": 97.5,  # 99.29% when set
    "sindr/src/composite.rs": 98.0,  # 99.50% when set
    "sindr/src/csv.rs": 98.5,  # 100.00% when set
    "sindr/src/datasheet.rs": 92.0,  # 93.83% when set
    "sindr/src/dc_sweep.rs": 98.5,  # 100.00% when set
    "sindr/src/design.rs": 98.0,  # 99.72% when set
    "sindr/src/device_params.rs": 98.5,  # 100.00% when set
    "sindr/src/element/ac_stamp.rs": 98.5,  # 100.00% when set
    "sindr/src/element/battery.rs": 96.0,  # 97.78% when set
    "sindr/src/element/behavioural.rs": 91.0,  # 92.55% when set
    "sindr/src/element/bjt.rs": 97.0,  # 98.99% when set
    "sindr/src/element/capacitor.rs": 98.5,  # 100.00% when set
    "sindr/src/element/cccs.rs": 88.0,  # 89.80% when set
    "sindr/src/element/ccvs.rs": 94.5,  # 96.49% when set
    "sindr/src/element/comparator.rs": 98.5,  # 100.00% when set
    "sindr/src/element/crystal.rs": 69.0,  # 70.59% when set
    "sindr/src/element/current_source.rs": 98.5,  # 100.00% when set
    "sindr/src/element/dc_motor.rs": 92.0,  # 93.67% when set
    "sindr/src/element/dc_stamp.rs": 98.5,  # 100.00% when set
    "sindr/src/element/digital_pin.rs": 92.5,  # 94.44% when set
    "sindr/src/element/diode.rs": 97.5,  # 99.14% when set
    "sindr/src/element/ferrite_bead.rs": 62.5,  # 64.29% when set
    "sindr/src/element/fuse.rs": 98.5,  # 100.00% when set
    "sindr/src/element/igbt.rs": 97.0,  # 98.84% when set
    "sindr/src/element/inductor.rs": 97.0,  # 98.89% when set
    "sindr/src/element/jfet.rs": 95.0,  # 96.96% when set
    "sindr/src/element/layout.rs": 98.5,  # 100.00% when set
    "sindr/src/element/led.rs": 98.5,  # 100.00% when set
    "sindr/src/element/logic_gate.rs": 83.5,  # 85.00% when set
    "sindr/src/element/mod.rs": 98.5,  # 100.00% when set
    "sindr/src/element/mosfet.rs": 97.5,  # 99.17% when set
    "sindr/src/element/op_amp.rs": 97.0,  # 98.90% when set
    "sindr/src/element/photodiode.rs": 89.5,  # 91.04% when set
    "sindr/src/element/photoresistor.rs": 88.5,  # 90.32% when set
    "sindr/src/element/potentiometer.rs": 91.5,  # 93.48% when set
    "sindr/src/element/pushbutton.rs": 98.5,  # 100.00% when set
    "sindr/src/element/relay.rs": 98.5,  # 100.00% when set
    "sindr/src/element/report.rs": 97.5,  # 99.29% when set
    "sindr/src/element/resistor.rs": 98.5,  # 100.00% when set
    "sindr/src/element/schottky_diode.rs": 98.5,  # 100.00% when set
    "sindr/src/element/switch.rs": 98.5,  # 100.00% when set
    "sindr/src/element/switch_spdt.rs": 53.0,  # 54.55% when set
    "sindr/src/element/switching_regulator.rs": 89.5,  # 91.10% when set
    "sindr/src/element/thermistor.rs": 90.0,  # 91.89% when set
    "sindr/src/element/tran_stamp.rs": 98.5,  # 100.00% when set
    "sindr/src/element/transformer.rs": 98.5,  # 100.00% when set
    "sindr/src/element/varactor.rs": 98.5,  # 100.00% when set
    "sindr/src/element/vccs.rs": 93.5,  # 95.08% when set
    "sindr/src/element/vcvs.rs": 98.5,  # 100.00% when set
    "sindr/src/element/voltage_regulator.rs": 96.5,  # 98.14% when set
    "sindr/src/element/voltage_source.rs": 98.5,  # 100.00% when set
    "sindr/src/element/zener_diode.rs": 95.0,  # 96.88% when set
    "sindr/src/error.rs": 97.5,  # 99.07% when set
    "sindr/src/examples.rs": 98.0,  # 99.80% when set
    "sindr/src/expression.rs": 95.5,  # 97.44% when set
    "sindr/src/fourier.rs": 92.5,  # 94.43% when set
    "sindr/src/handle.rs": 89.0,  # 90.56% when set
    "sindr/src/import/bench.rs": 98.5,  # 100.00% when set
    "sindr/src/import/mod.rs": 98.0,  # 99.68% when set
    "sindr/src/import/subckt.rs": 97.0,  # 98.65% when set
    "sindr/src/import/value.rs": 97.5,  # 99.28% when set
    "sindr/src/integration.rs": 98.5,  # 100.00% when set
    "sindr/src/lib.rs": 95.5,  # 97.20% when set
    "sindr/src/loop_gain.rs": 94.0,  # 95.67% when set
    "sindr/src/measure.rs": 97.5,  # 99.17% when set
    "sindr/src/mna.rs": 97.0,  # 98.78% when set
    "sindr/src/newton_raphson.rs": 97.0,  # 98.69% when set
    "sindr/src/node_map.rs": 98.5,  # 100.00% when set
    "sindr/src/noise.rs": 97.5,  # 99.22% when set
    "sindr/src/op_amp.rs": 96.5,  # 98.35% when set
    "sindr/src/pole_zero.rs": 90.5,  # 92.04% when set
    "sindr/src/ratings.rs": 97.5,  # 99.43% when set
    "sindr/src/results.rs": 91.5,  # 93.49% when set
    "sindr/src/sensitivity.rs": 95.5,  # 97.22% when set
    "sindr/src/sensitivity/ac.rs": 92.5,  # 94.24% when set
    "sindr/src/sensitivity/device.rs": 97.0,  # 98.87% when set
    "sindr/src/session.rs": 98.0,  # 99.45% when set
    "sindr/src/solver.rs": 98.5,  # 100.00% when set
    "sindr/src/spice/analysis.rs": 98.0,  # 99.79% when set
    "sindr/src/spice/ast.rs": 98.5,  # 100.00% when set
    "sindr/src/spice/build.rs": 91.0,  # 92.60% when set
    "sindr/src/spice/error.rs": 87.0,  # 88.89% when set
    "sindr/src/spice/export.rs": 98.0,  # 99.81% when set
    "sindr/src/spice/flatten.rs": 96.5,  # 98.12% when set
    "sindr/src/spice/mod.rs": 97.0,  # 98.77% when set
    "sindr/src/spice/param_eval.rs": 96.0,  # 97.89% when set
    "sindr/src/spice/parser.rs": 95.0,  # 96.70% when set
    "sindr/src/spice/preprocess.rs": 95.5,  # 97.26% when set
    "sindr/src/spice/si.rs": 98.5,  # 100.00% when set
    "sindr/src/spice/source_map.rs": 96.0,  # 97.87% when set
    "sindr/src/stamp.rs": 96.5,  # 98.04% when set
    "sindr/src/temp_sweep.rs": 96.5,  # 98.32% when set
    "sindr/src/thermal.rs": 94.5,  # 96.02% when set
    "sindr/src/transfer_function.rs": 94.5,  # 96.32% when set
    "sindr/src/transient.rs": 97.5,  # 99.06% when set
    "sindr/src/validation.rs": 96.5,  # 98.29% when set
    "sindr/src/variation.rs": 95.5,  # 97.27% when set
    "sindr/src/waveform.rs": 97.0,  # 98.88% when set
    "TOTAL": 96.5,  # 98.12% when set
}


def line_coverage(path: str) -> dict[str, float]:
    out = {}
    with open(path, encoding="utf-8") as f:
        for row in f:
            cols = row.split()
            # Filename, then regions (3), functions (3), lines (3), branches (3).
            if len(cols) >= 10 and cols[9].endswith("%"):
                name = cols[0].replace("\\", "/")
                for key in FLOORS:
                    if name == key or name.endswith("/" + key):
                        out[key] = float(cols[9].rstrip("%"))
    return out


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    found = line_coverage(sys.argv[1])
    failed = False
    print("| file | lines | floor |")
    print("|---|---:|---:|")
    for name, floor in FLOORS.items():
        if name not in found:
            print(f"| {name} | missing | {floor:.1f}% |")
            failed = True
            continue
        mark = "" if found[name] >= floor else " ❌"
        failed |= found[name] < floor
        print(f"| {name} | {found[name]:.2f}% | {floor:.1f}%{mark} |")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
