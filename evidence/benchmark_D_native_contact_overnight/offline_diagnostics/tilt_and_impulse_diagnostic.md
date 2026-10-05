# Benchmark D offline tilt and impulse diagnostic

- Tilt classification: **TILT_PRIMARILY_PREEXISTING**. The 6.25 µs branch is already at 23.073° immediately before the first response; the first completed response adds only 0.120°.
- The robot begins the saved interval at 20.105° and reaches 24.826° at 1.900 ms, so most visible tilt is accumulated before the native proximity response.
- Peak angular rate is 872.900 rad/s, while the first response changes the recorded angular-rate magnitude from 872.900 to 339.631 rad/s.
- Frozen 12.5 vs 6.25 µs convergence: **FAIL remains unchanged** (7 vs 14 impulses; allowed difference 3).
- One response episode spans 1.81250–1.90000 ms in both branches. Halving dt doubles the raw callback impulse count and reduces the median individual impulse, while cumulative normal impulse differs by 0.1544% and cumulative angular impulse by 0.0615%.
- Impulse interpretation: **RAW_COUNT_TIMESTEP_ARTIFACT_LIKELY**. This is a physical interpretation of the saved episode, not a retroactive change to the frozen gate.
- Final exact-common-time trajectory differences are COM 2.06354e-07 m, tilt 0.0143886°, velocity 0.000432787 m/s, and angular rate 4.11407 rad/s.
- Visuals cover 1.750–1.900 ms and are diagnostic only; they are not a final 2.000-ms production result. The saved first response is a native near-wall proximity/contact response at the ~100 µm threshold, not an ideal zero-gap impact.
