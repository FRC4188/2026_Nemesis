package frc.robot.drivecontrol;

import edu.wpi.first.math.geometry.Rotation2d;

/** Immutable drive intent. Auto-path execution remains a placeholder in the hybrid phase. */
public final class DriveRequest {
  public final DriveControlState wantedState;
  public final DriveOwner owner;
  public final double xPercent;
  public final double yPercent;
  public final double omegaPercent;
  public final boolean hasDriveInput;
  public final boolean headingLock;
  public final HeadingTargetType headingTargetType;
  public final Rotation2d explicitHeadingTarget;
  public final boolean allowInterrupt;
  public final DriveGoal goal;

  public DriveRequest(
      DriveControlState wantedState,
      DriveOwner owner,
      double xPercent,
      double yPercent,
      double omegaPercent,
      boolean hasDriveInput,
      boolean headingLock,
      HeadingTargetType headingTargetType,
      Rotation2d explicitHeadingTarget,
      boolean allowInterrupt,
      DriveGoal goal) {
    this.wantedState = wantedState;
    this.owner = owner;
    this.xPercent = xPercent;
    this.yPercent = yPercent;
    this.omegaPercent = omegaPercent;
    this.hasDriveInput = hasDriveInput;
    this.headingLock = headingLock;
    this.headingTargetType = headingTargetType;
    this.explicitHeadingTarget = explicitHeadingTarget;
    this.allowInterrupt = allowInterrupt;
    this.goal = goal;
  }

  public static DriveRequest idle() {
    return new DriveRequest(
        DriveControlState.IDLE_X,
        DriveOwner.NONE,
        0.0,
        0.0,
        0.0,
        false,
        false,
        HeadingTargetType.NONE,
        Rotation2d.kZero,
        true,
        DriveGoal.none());
  }

  public static DriveRequest teleopOpenLoop(double x, double y, double omega) {
    return new DriveRequest(
        DriveControlState.TELEOP_OPEN_LOOP,
        DriveOwner.TELEOP,
        x,
        y,
        omega,
        x != 0.0 || y != 0.0 || omega != 0.0,
        false,
        HeadingTargetType.NONE,
        Rotation2d.kZero,
        true,
        DriveGoal.none());
  }

  public static DriveRequest teleopHeadingLock(
      double x, double y, HeadingTargetType targetType, Rotation2d explicitTarget) {
    return new DriveRequest(
        DriveControlState.TELEOP_HEADING_LOCK,
        DriveOwner.TELEOP,
        x,
        y,
        0.0,
        true,
        true,
        targetType,
        explicitTarget,
        true,
        DriveGoal.none());
  }

  public static DriveRequest autoPath(DriveGoal goal) {
    return new DriveRequest(
        DriveControlState.AUTO_PATH_FOLLOWING,
        DriveOwner.AUTO,
        0.0,
        0.0,
        0.0,
        true,
        false,
        HeadingTargetType.NONE,
        Rotation2d.kZero,
        true,
        goal);
  }

  public static DriveRequest characterization(double volts) {
    return new DriveRequest(
        DriveControlState.CHARACTERIZATION,
        DriveOwner.CHARACTERIZATION,
        0.0,
        0.0,
        0.0,
        true,
        false,
        HeadingTargetType.NONE,
        Rotation2d.kZero,
        false,
        new DriveGoal(null, volts));
  }
}
