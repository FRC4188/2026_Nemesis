# Migration status

| Step | Status | Notes |
| --- | --- | --- |
| 0. Inventory | Complete | Initial catalogs and risk/equivalence maps captured before code edits |
| 1. Shared state-machine utilities | Complete | Typed requests, transitions, timing, hooks, validation, and logging |
| 2. Intake internal state | Complete | Existing public methods retained |
| 3. Hopper internal state | Complete | Existing public methods retained; `setRPM` behavior preserved |
| 4. Shooter internal state | Complete | Existing public methods and one-sided `atGoal` retained |
| 5. Hood internal state | Complete | Offset and public APIs retained |
| 6. Wrist internal state | Complete | Public APIs retained; timed force-down owned by Superstructure |
| 7-10. Superstructure | Complete | Shooting, aiming, intake/eject, concurrent intake/wrist lanes, and logs implemented |
| 11. Selected teleop scoring bindings | Complete | Original trigger edges and requirements retained |
| 12-16. Teleop DriveController | Complete | `joystickCombined` retained for compatibility; teleop binding switched |
| 17. Auto mechanism requests | Complete for scoring mechanisms | Scoring/tuning, slow-up/force-down, and exact auto intake voltages request Superstructure; explicit one-shot stops remain lifecycle commands |
| 18. Auto path state | Scaffolded/deferred | Placeholder request/state exists; path output deliberately remains command-based until on-robot teleop equivalence is proven |
| 19. Full auto routine state machine | Optional/deferred | Explicitly last in the migration plan |

Verification uses the installed WPILib JDK 17 because the shell default is Java 8. `compileJava`
and the migration unit tests pass. The final verification pass also runs the complete Gradle build,
format checks, and diff checks.
