# Resource map

| Resource | Current writers | Hybrid owner |
| --- | --- | --- |
| Drive velocity | Teleop drive command, auto path commands, `DriveToPose`, characterization | `DriveController` for migrated teleop only; commands retain auto/characterization ownership |
| Drive vision acceptance | `joystickCombined` | `DriveController` for migrated teleop; legacy auto heading command remains unchanged |
| Hood output | Scoring aim commands | `Superstructure` for migrated scoring requests |
| Shooter output | Scoring shoot/data commands | `Superstructure` for static/manual/pass/tuning requests |
| Hopper output | Scoring shoot and eject commands | `Superstructure` for migrated shoot/eject requests |
| Intake output | Teleop intake/eject and many auto intake commands | `Superstructure`; request goals preserve each teleop/auto voltage |
| Wrist output | Scoring wrist commands | `Superstructure` for manual/slow-up/down/good-stow/force-down |
| Vision processing enable | Copilot toggle | Vision subsystem command remains unchanged |

During the hybrid phase, WPILib requirements remain on wrapper commands. A wrapper requests a
state and never directly writes the resource governed by that request.
