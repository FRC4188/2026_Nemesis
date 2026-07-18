# Trigger catalog

| Input | Binding | Existing semantics |
| --- | --- | --- |
| Pilot Start | `onTrue` pose rotation reset | Runs while disabled; requires Drive |
| Composite drive input | `whileTrue joystickCombined` | Left/right stick or RB/A/X; requires Drive; stop-with-X on falling edge/interruption |
| Pilot RB | `whileTrue staticAim` | Static hood aim |
| Pilot A | `whileTrue passAim` | 40 degree pass hood aim |
| Pilot Y or B | `whileTrue manualAim` | 12 ft for Y, 3.5 ft while B |
| Pilot right trigger | `whileTrue either(...)` | Pass shoot if A, static shoot if RB, otherwise manual shoot |
| Pilot left trigger | `whileTrue` | Intake +8.75 V; stop on release |
| Pilot left bumper | `whileTrue parallel` | Hopper -6 V/0 RPM plus intake eject +6 V; stop both on release |
| Copilot Y | `whileTrue` | Live manual wrist voltage `3 * -leftY`; stop on release |
| Copilot X | `onTrue downNoStall` | Wrist -4 V until below 30 degrees |
| Copilot A | `onTrue goodStow` | Wrist +5 V until above 120 degrees |
| Copilot RB | `onTrue forceDown` | Three-step timed wrist action |
| Copilot left trigger | `toggleOnTrue` | Disable shake while toggled command is active; restore on end |
| Copilot POV right | `onTrue` | Wrist zero |
| Copilot POV up/down | `onTrue` | Hood offset +1/-1 |
| Copilot right trigger | `toggleOnTrue` | Disable/enable vision processing |

The composite drive condition is exactly: nonzero corrected left stick, nonzero corrected right X,
RB, A, or X. Translation uses `Scale.SQUARED`; rotation uses `Scale.WILL`.
