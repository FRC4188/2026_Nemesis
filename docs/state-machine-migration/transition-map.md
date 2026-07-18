# Transition map

## Superstructure

- Disabled forces `IDLE` and stops all coordinated mechanisms.
- `STATIC_SHOOTING`, `MANUAL_SHOOTING`, and `PASS_SHOOTING` enter through their corresponding
  aiming states.
- Aim-only requests remain in aiming and never feed.
- Shoot requests remain in aiming for at least 0.1 seconds and until `shooter.atGoal()`, then enter
  the matching shooting state.
- Shooting runs the hopper at 9.0 V/5000 RPM. After 0.1 seconds and `hopper.indexAtGoal()`, the
  initial-shot offset is cleared.
- Wrist force-down enters step 1, advances to step 2 after 0.12 seconds, advances to step 3 after
  another 0.12 seconds, and finishes below 30 degrees.
- Wrist down-no-stall finishes below 30 degrees; good-stow finishes above 120 degrees.
- Clearing the active held request returns to `IDLE` and performs state-specific stop behavior.
- Intake and wrist requests use independent lanes so they can remain concurrent with scoring when
  their original WPILib requirements are disjoint. Intake request goals carry the exact voltage.

## DriveController

- Disabled forces `DISABLED`.
- An active teleop request without heading buttons enters `TELEOP_OPEN_LOOP`.
- RB/A/X heading requests enter `TELEOP_HEADING_LOCK`; the angle controller resets only on entry.
- Ending the drive wrapper requests `IDLE_X`, calls stop-with-X, and restores vision acceptance.
- Auto path and characterization states are declared for ownership completeness but stay under
  command ownership in the first hybrid phase.
- Teleop requests outside teleop are rejected; auto requests outside autonomous are rejected.
- In autonomous with no DriveController auto request, `IDLE_X` deliberately yields outputs to the
  still-command-based path and characterization commands.
