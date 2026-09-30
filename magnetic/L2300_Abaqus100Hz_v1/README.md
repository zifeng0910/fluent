# L2300 Abaqus 100 Hz source recovery

This directory is a validated analytic baseline, not an approved physical Magpylib lookup. Benchmark C remains closed.

The successful case is TRUECEL_B0P11_G2P20_F100_NOFLUID_CONTROL (50 ms, five cycles). Executed VUAMP and its actual table take precedence over metadata and helper equations. The physical magnetic source is not uniquely recoverable. No sphere or equivalent fitted magnet was created.

## Reproduce

Use H:/fluent/.venv/Scripts/python.exe with generate_magnetic_lookup.py, then validate_abaqus100Hz_crosscheck.py. Dependencies: numpy, scipy, PyYAML, Magpylib, ezdxf. Regeneration reads the recorded historical source paths on J:. Runtime uses only the files in this directory. Production code uses its analytic branch; Magpylib is imported by that legacy implementation but no physical source or getB calculation is substituted.

## Conventions

Inputs: current COM xyz in metres, unit quaternion xyzw covering all SO(3), absolute time 0..50 ms. Old tube basis is rigidly re-expressed as Fluent XYZ. Old RP/mass/inertia/geometry/moment are not inherited. Magnetic force is global N and torque is global N*m about current COM. Periodic phase factors are continuously interpolated; absolute time retains startup and translating driver. Out-of-domain positions/time/invalid orientations cause hard stop. The old analytic model has no transverse position dependence.

L2300 magnetic moment direction is recovered from its executed regression and inverse CAD placement, not assumed from +X. It is approximately +X while the successful old robot was -X. The unchanged gradient consequently reverses force polarity; this is explicitly reported in the load comparison.

The helper table_model.field_unit has a sign/frame mismatch with the executed table. Generation samples the actual production analytic implementation, which reproduces the successful table at machine precision.
