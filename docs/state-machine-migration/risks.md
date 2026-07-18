# Migration risks and follow-ups

## Preserved suspected defects

- `Hopper.indexAtGoal()` compares measured RPM against `setRPM`, but `runHopper()` does not update
  `setRPM`. The migration intentionally does not repair this behavior.
- `Shooter.atGoal()` uses a one-sided error comparison rather than absolute error. This is
  preserved.
- `Paths.getFirstPose()` checks `isEmpty()` and then calls `get()` in the empty branch. This is
  outside the migration and is not changed.
- `RobotContainer.getAutonomousCommand()` returns `VoyagerLib.runSelectedAuto()` even though a
  separate `autoChooser` is populated. This selection behavior is not changed.

## High-risk lifecycle areas

- Parallel aim/shoot requests must select shooting without losing the still-held aim request when
  shooting ends.
- `initialShots` must reset on cancellation and timeout.
- State-machine periodic execution occurs after `CommandScheduler.run()` so wrapper requests are
  applied in the same robot loop.
- Auto `autoShoot` uses the legacy `joystickCombined` command during autonomous; it must not be
  routed through teleop-only DriveController validation.
- PathPlanner, PathBuilder, Voyager, CSP pathing, event markers, command deadlines, conditionals,
  and characterization remain command-owned.
- The local default Java runtime is Java 8. GradleRIO 2026 requires Java 17+; verification uses the
  installed WPILib 2024 JDK (Temurin 17.0.8.1).
- Full physical equivalence cannot be established in desktop simulation. Teleop heading feel,
  vision accept/reject timing, shooter/hopper readiness, and wrist thresholds need robot testing
  before auto path ownership is moved.
