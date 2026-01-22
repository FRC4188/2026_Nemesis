package frc.robot.CSPLib.ppp;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

public final class PathBuilder {
  private static Drive drive;
  private static PathConstraints constraints =
      new PathConstraints(
          Constants.Drive.DRIVE_MAXVEL,
          Constants.Drive.DRIVE_MAXACC,
          Constants.Drive.ANGLE_MAXVEL,
          Constants.Drive.ANGLE_MAXACC);

  public static void configure(Drive drivetrain) {
    if (drive != null) return;
    drive = drivetrain;

    AutoBuilder.configure(
        PathBuilder::getPose,
        drive::setPose,
        PathBuilder::getChassisSpeeds,
        PathBuilder::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                Constants.Drive.DRIVE_PID.getP(),
                Constants.Drive.DRIVE_PID.getI(),
                Constants.Drive.DRIVE_PID.getD()),
            new PIDConstants(
                Constants.Drive.ANGLE_PID.getP(),
                Constants.Drive.ANGLE_PID.getI(),
                Constants.Drive.ANGLE_PID.getD())),
        Constants.Drive.PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public static void runVelocity(ChassisSpeeds speeds) {
    switch (Constants.Robot.robotMode) {
      case SHOOT:
        drive.runVelocityOffset(speeds, Constants.Shooter.location);
        break;
      default:
        drive.runVelocity(speeds);
    }
  }

  public static ChassisSpeeds getChassisSpeeds() {
    switch (Constants.Robot.robotMode) {
      case SHOOT:
        return drive.getChassisSpeedsOffset(Constants.Shooter.location);
      default:
        return drive.getChassisSpeeds();
    }
  }

  public static Pose2d getPose() {
    switch (Constants.Robot.robotMode) {
      case SHOOT:
        return drive.getPoseOffset(Constants.Shooter.location);
      default:
        return drive.getPose();
    }
  }

  // Tracks a translation2d on the field
  public static void targetTranslation(Supplier<Translation2d> wanted) {
    targetRotation(() -> wanted.get().minus(drive.getPose().getTranslation()).getAngle());
  }

  // Tracks a set rotation, approaches it
  public static void targetRotation(Supplier<Rotation2d> wanted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Supplier<Rotation2d> rotationSupplier = wanted;
          Constants.Drive.ANGLE_PID.enableContinuousInput(-Math.PI, Math.PI);

          Logger.recordOutput(
              "PathBuilder/Track Target Angle", rotationSupplier.get().getRadians());
          Logger.recordOutput(
              "PathBuilder/Track Current Angle", drive.getPose().getRotation().getRadians());

          double omega =
              Constants.Drive.ANGLE_PID.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians())
                  + Constants.Drive.ANGLE_PID.getSetpoint().velocity * Constants.Drive.ANGLE_FF;

          if (Math.abs(drive.getRotation().getRadians() - rotationSupplier.get().getRadians())
                  < Constants.Drive.ANGLE_TOL
              && Constants.Drive.ANGLE_PID.getSetpoint().velocity == 0.0) omega = 0.0;

          return omega;
        });
  }

  // Stops any tracking happening
  public static void stopTarget() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  // get path constraints
  public static PathConstraints getConstraints() {
    return constraints;
  }

  // set path constraints, if a change is necessary
  public static void setConstraints(PathConstraints constr) {
    constraints = constr;
  }

  // Good but slow, need fast seamless paths
  public static Command createPath(Pose2d endPose) {
    return AutoBuilder.pathfindToPose(endPose, constraints, 0.0).finallyDo(() -> drive.stop());
  }

  public static Command createPath(Pose2d... poses) {
    return Commands.sequence(
            IntStream.range(0, poses.length)
                .mapToObj(
                    i ->
                        i == poses.length - 1
                            ? PathBuilder.createPath(poses[i], 0)
                            : PathBuilder.createPath(poses[i], 3.0))
                .toArray(Command[]::new))
        .finallyDo(() -> drive.stop());
  }

  public static Command createPath(Translation2d... translations) {
    return Commands.sequence(
            IntStream.range(0, translations.length)
                .mapToObj(
                    i ->
                        i == translations.length - 1
                            ? PathBuilder.createPath(translations[i], 0)
                            : PathBuilder.createPath(translations[i], 3.0))
                .toArray(Command[]::new))
        .finallyDo(() -> drive.stop());
  }

  public static Command createPath(Translation2d endTranslation) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(endTranslation, new Rotation2d()), constraints, 0.0)
        .beforeStarting(() -> Constants.Drive.ANGLE_PID.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }

  public static Command createPath(Pose2d endPose, double endVel) {
    return AutoBuilder.pathfindToPose(endPose, constraints, endVel).finallyDo(() -> drive.stop());
  }

  public static Command createPath(Translation2d endTranslation, double endVel) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(endTranslation, new Rotation2d()), constraints, endVel)
        .beforeStarting(() -> Constants.Drive.ANGLE_PID.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }

  public static Command mergeToPath(PathPlannerPath knownPath) {
    return AutoBuilder.pathfindThenFollowPath(knownPath, constraints)
        .beforeStarting(() -> Constants.Drive.ANGLE_PID.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }
}
