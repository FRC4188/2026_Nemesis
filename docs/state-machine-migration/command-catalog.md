# Command catalog

## ScoringCommands

| Command | Existing requirements | Lifecycle/output behavior |
| --- | --- | --- |
| `dataShoot` | Shooter, Hopper | Live tuning RPM request; 0.1 s wait; shooter-ready gate; feed until interrupted |
| `staticAim` | Hood | Live hub distance and hood formula; stop hood on end |
| `staticShoot` | Shooter, Hopper | Live hub distance and RPM formula; 0.1 s shooter delay; shooter and hopper gates; reset `initialShots` on end |
| `manualAim` | Hood | Samples live feet supplier and converts to meters; stop hood on end |
| `manualShoot` | Shooter, Hopper | Samples live feet supplier; same shooting lifecycle with +300 initial offset |
| `passAim` | Hood | 40 degree hood target; stop hood on end |
| `passShoot` | Shooter, Hopper | Live alliance-flipped X RPM; 0.1 s shooter delay; feed until interrupted |
| `slowUp` | Wrist, Intake when enabled | Size-dependent state delay, wrist +4 V to 90 degrees, intake +5 V; skipped when shake disabled |
| `downNoStall` | Wrist | -4 V until angle below 30 degrees; stop on end |
| `forceDown` | Wrist | -6 V/0.12 s, +8 V/0.12 s, then -8 V until below 30 degrees; stop finally |
| `goodStow` | Wrist | +5 V until above 120 degrees; stop on end |

## Drive commands

| Command | Requirements | Lifecycle/output behavior |
| --- | --- | --- |
| `joystickCombined` | Drive | Field-relative open loop or heading lock; reset angle PID before start; stop-with-X and restore vision on end |
| `feedforwardCharacterization` | Drive during output phases | Orient, ramp volts, collect and print regression samples |
| `wheelRadiusCharacterization` | Drive during output phase | Slew-rate turn command, measure wheel/gyro delta, print radius |
| `DriveToPose` | Drive | Profiled translation/heading output; stop-with-X on end |

## Autonomous commands

Public routine factories are `autoShoot`, `newAuto`, `doubleSwipeBothBump`, `follower`,
`pseudoBoard`, `rightDisrupt`, `leftDisrupt`, `disruptDoubleSwipe`, and `fullDepot`.

`AutoCommands` contains command sequences/deadlines, conditional branches, timeouts, PathPlanner
followers, PathBuilder paths and triggers, intake voltage request wrappers, wrist force-down request
wrappers, and the shared `autoShoot` group. These remain command-sequenced during the hybrid
migration.

## Default commands

No `setDefaultCommand` registrations were found. Teleop drive is scheduled by the `driveInput`
trigger instead.
