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
import edu.wpi.first.math.util.Units;
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
  private static boolean currentlyTracking = false;
  private static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private PathBuilder() {}

  public static void instantiate(Drive drivetrain) {
    if (drive != null) return;

    drive = drivetrain;

    AutoBuilder.configure(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        speeds ->
            drive.runVelocityOffset(
                speeds,
                (currentlyTracking) ? Constants.Robot.CENTER_OF_ROTATION : new Translation2d(0, 0)),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        Drive.PP_CONFIG,
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

  public static void trackTranslation(Supplier<Translation2d> wanted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    currentlyTracking = true;

    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Supplier<Rotation2d> rotationSupplier =
              () -> wanted.get().minus(drive.getPose().getTranslation()).getAngle();
          drive.angleController.enableContinuousInput(-Math.PI, Math.PI);

          Logger.recordOutput(
              "PathBuilder/Track Target Angle", rotationSupplier.get().getRadians());
          Logger.recordOutput(
              "Drive/Track Angle Current", drive.getPose().getRotation().getRadians());

          double omega =
              drive.angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians())
                  + drive.angleController.getSetpoint().velocity * Constants.Robot.ANGLE_FF;

          if (Math.abs(drive.getRotation().getRadians() - rotationSupplier.get().getRadians())
                  < Constants.Robot.ANGLE_TOL
              && drive.angleController.getSetpoint().velocity == 0.0) omega = 0.0;

          return omega;
        });
  }

  public static void trackRotation(Supplier<Rotation2d> wanted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    currentlyTracking = true;

    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Supplier<Rotation2d> rotationSupplier = wanted;
          drive.angleController.enableContinuousInput(-Math.PI, Math.PI);

          Logger.recordOutput(
              "PathBuilder/Track Target Angle", rotationSupplier.get().getRadians());
          Logger.recordOutput(
              "Drive/Track Angle Current", drive.getPose().getRotation().getRadians());

          double omega =
              drive.angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians())
                  + drive.angleController.getSetpoint().velocity * Constants.Robot.ANGLE_FF;

          if (Math.abs(drive.getRotation().getRadians() - rotationSupplier.get().getRadians())
                  < Constants.Robot.ANGLE_TOL
              && drive.angleController.getSetpoint().velocity == 0.0) omega = 0.0;

          return omega;
        });
  }

  public static void stopTrack() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    currentlyTracking = false;
  }

  public static PathConstraints getConstraints() {
    return constraints;
  }

  public static void setConstraints(PathConstraints constr) {
    constraints = constr;
  }

  public static Command driveWithBuiltPath(Pose2d endPose) {
    return AutoBuilder.pathfindToPose(endPose, constraints, 0.0).finallyDo(() -> drive.stop());
  }

  public static Command driveWithBuiltPath(Pose2d... poses) {
    return Commands.sequence(
            IntStream.range(0, poses.length)
                .mapToObj(
                    i ->
                        i == poses.length - 1
                            ? PathBuilder.driveWithBuiltPath(poses[i], 0)
                            : PathBuilder.driveWithBuiltPath(poses[i], 3.0))
                .toArray(Command[]::new))
        .finallyDo(() -> drive.stop());
  }

  public static Command driveWithBuiltPath(Translation2d... translations) {
    return Commands.sequence(
        IntStream.range(0, translations.length)
            .mapToObj(
                i ->
                    i == translations.length - 1
                        ? PathBuilder.driveWithBuiltPath(translations[i], 0)
                        : PathBuilder.driveWithBuiltPath(translations[i], 3.0))
            .toArray(Command[]::new));
  }

  public static Command driveWithBuiltPath(Translation2d endTranslation) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(endTranslation, new Rotation2d()), constraints, 0.0)
        .beforeStarting(() -> drive.angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }

  public static Command driveWithBuiltPath(Pose2d endPose, double endVel) {
    return AutoBuilder.pathfindToPose(endPose, constraints, endVel).finallyDo(() -> drive.stop());
  }

  public static Command driveWithBuiltPath(Translation2d endTranslation, double endVel) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(endTranslation, new Rotation2d()), constraints, endVel)
        .beforeStarting(() -> drive.angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }

  public static Command mergeToKnownPath(PathPlannerPath knownPath) {
    return AutoBuilder.pathfindThenFollowPath(knownPath, constraints)
        .beforeStarting(() -> drive.angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }
}
