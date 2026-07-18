# Autonomous catalog

## Dashboard/configured entries

- FORWARD Right Disruptor
- FORWARD Left Disruptor
- Nothing
- Right Follower
- Left Follower
- Left Double Bump
- Right Double Bump
- CSP path test
- 1323 match
- Drive Wheel Radius Characterization
- Drive Simple FF Characterization

Voyager events are `Delay` (4.0 s), `Shoot Half` (2.0 s timeout), and `Shoot Full` (4.0 s
timeout).

## Routine factories

- `autoShoot(Size)` combines legacy heading-lock drive, 1.5 V intake for 1 second, static aiming,
  static shooting, and size-dependent wrist/intake behavior.
- `newAuto(Start, Cycle)` builds NONE, NZ, and DOUBLE cycles from PathPlanner followers,
  PathBuilder distance triggers, intake commands, force-down, and timed shooting groups.
- `doubleSwipeBothBump(Start)` and `follower(Start)` compose PathPlanner paths and scoring actions.
- `pseudoBoard(Start, Swipe, Cycle)` constructs large PathBuilder-based conditional routines.
- `rightDisrupt` and `leftDisrupt` use PathBuilder paths/triggers and finish with timed shooting.
- `disruptDoubleSwipe` and `fullDepot` are additional PathBuilder routines.

## Compatibility boundary

`CommandScheduler`, `AutoBuilder`, PathBuilder, Voyager, `CSPPath`, command groups, timeouts, and
event markers remain unchanged. Auto path output is not moved into `DriveController` during the
first hybrid phase. `Robot.teleopInit` continues cancelling the scheduled autonomous command.
