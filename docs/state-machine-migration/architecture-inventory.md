# Architecture inventory

Inventory captured before migration on 2026-07-17.

## Runtime structure

- `Robot` starts AdvantageKit, constructs `RobotContainer`, runs `CommandScheduler` from
  `robotPeriodic`, schedules the selected autonomous command in `autonomousInit`, and cancels it in
  `teleopInit`.
- `RobotContainer` constructs the singleton subsystems, configures Voyager, PathBuilder, and
  `CSPPath`, registers controller triggers, and publishes match-state logs.
- `Drive`, `Vision`, `Intake`, `Hopper`, `Shooter`, `Hood`, and `Wrist` are WPILib subsystems and own
  their hardware IO.
- `ScoringCommands` directly writes scoring subsystem outputs.
- `DriveCommands.joystickCombined` directly writes drive velocity and vision-acceptance state.
- `AutoCommands` remains the autonomous sequence owner and composes PathPlanner, PathBuilder,
  scoring, intake, and wrist commands.

## Subsystems and public output APIs

| Subsystem | Output APIs currently used |
| --- | --- |
| Drive | `runVelocity`, `runCharacterization`, `stop`, `stopWithX`, `acceptVision`, `setPose` |
| Vision | `enableVision` |
| Intake | `intakeVolts`, `ejectVolts`, `stop` |
| Hopper | `runHopper`, `stop`, `indexAtGoal` |
| Shooter | `setVelocityRPM`, `runTC`, `stop`, `atGoal` |
| Hood | `runHoodVolts`, `setAngle`, `stow`, `zero`, `addOne`, `subOne`, `stop` |
| Wrist | `runWristVolts`, `stow`, `down`, `setAngle`, `zero`, `enableShake`, `stop` |

## Existing behavior coordinators

- Scoring: `ScoringCommands` and command groups in `AutoCommands`.
- Teleop drive: `DriveCommands.joystickCombined`.
- Auto paths: PathPlanner `AutoBuilder`, the imported `frc.lib.pathbuilder.PathBuilder`, Voyager,
  and `CSPPath`.
- Match lifecycle: `Robot` plus controller triggers registered by `RobotContainer`.

## Migration boundary

The hybrid migration keeps command scheduling, autonomous path sequencing, event markers, and
characterization command-based. `Superstructure` becomes the scoring output coordinator and
`DriveController` becomes the teleop drive output coordinator. Hardware IO remains in the existing
subsystems.

To preserve the old disjoint command requirements, the Superstructure has coordinated primary
scoring behavior plus independent intake and wrist lanes. This allows, for example, hood/shooter
aiming, intake, and a wrist action to remain concurrent without creating a combined global enum.
