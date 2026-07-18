package frc.robot.drivecontrol;

import com.pathplanner.lib.path.PathPlannerPath;

/** Context carried by future auto-path and characterization requests. */
public final class DriveGoal {
  public final PathPlannerPath autoPath;
  public final double characterizationVolts;

  public DriveGoal(PathPlannerPath autoPath, double characterizationVolts) {
    this.autoPath = autoPath;
    this.characterizationVolts = characterizationVolts;
  }

  public static DriveGoal none() {
    return new DriveGoal(null, 0.0);
  }
}
