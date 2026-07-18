package frc.robot.drivecontrol;

public enum DriveControlState {
  DISABLED,
  IDLE_X,
  TELEOP_OPEN_LOOP,
  TELEOP_HEADING_LOCK,
  AUTO_PATH_FOLLOWING,
  AUTO_ALIGN,
  CHARACTERIZATION,
  FAULTED
}
