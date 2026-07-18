# Equivalence matrix

| Behavior | Before | Hybrid target | Verification |
| --- | --- | --- | --- |
| Static hood | `inclineHueristic(live hub distance)` | Same formula and live pose | Source review + build |
| Static shooter | `RPMRegress(distance) + 200` initially | Same | Source review + state tests/build |
| Manual shooter | Feet supplier converted to meters, +300 initially | Same | Source review + state tests/build |
| Pass aim/shoot | 40 degrees; `110 * metersToFeet(flipped X)` | Same | Source review + build |
| Shooter feed gate | 0.1 s then `shooter.atGoal()` | FPGA time + same gate | Source review |
| Initial shots | Reset true on end; false after 0.1 s feed and hopper gate | Same | Source review |
| Teleop intake/eject | +8.75 V; eject +6 V and hopper -6 V/0 RPM | Same | Source review |
| Wrist force-down | -6/+8/-8 V, 0.12/0.12 s, angle under 30 | Same FPGA-time sequence | Source review |
| Drive scaling/signs | SQUARED translation, WILL omega, existing negations | Same suppliers | Source review |
| Heading priority | X, then A, then RB/hub | Same | Source review |
| Heading output | Shared profiled PID, field-relative drive, vision rules | Same | Source review |
| Drive end | stop-with-X and `acceptVision(true)` | Same | Source review |
| Autonomous scheduling | Command selected and scheduled; cancelled at teleop | Unchanged | Source review + build |
| Path/characterization | Command-owned | Unchanged | Source review + build |

Automated verification covers scoring formulas, typed request goals and auto origin, intake voltage
goals, open-loop versus heading-lock requests, idle ownership, and X/A/hub heading priority. Final
on-robot verification is still required for motor direction, sensor timing, and physical control
feel.
