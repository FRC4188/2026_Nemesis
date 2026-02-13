package frc.robot.CSPLib.ppp;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CSPLib.util.ProjMath;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;

/**
 * A CSP-made class utilizing PathPlanner AutoBuilder, allowing us to create paths, follow paths,
 * and get drive properties based on the robot mode.
 */
public final class PathBuilder {
  private static Drive drive;
  public static FollowPath.Builder pathBuilder;

  private static Supplier<Rotation2d> trackingSupplier;

  // Add Multiplier if too fast
  private static PathConstraints constraints =
      new PathConstraints(
          Constants.Drive.DRIVE_MAXVEL * 0.5,
          Constants.Drive.DRIVE_MAXACC * 0.5,
          Constants.Drive.ANGLE_MAXVEL * 0.5,
          Constants.Drive.ANGLE_MAXACC * 0.5);

  /**
   * A method to configure the PathBuilder class, setting it up with the Drivetrain instance.
   * Enables PathFinding and AutoBuilder.
   *
   * @param drivetrain
   */
  public static void configure(Drive drivetrain) { // Add parameters for ALL subsystems
    if (drive != null) return;
    drive = drivetrain;

    pathBuilder =
        new FollowPath.Builder(
                drive,
                drive::getPose,
                PathBuilder::getChassisSpeeds,
                PathBuilder::runVelocity,
                new PIDController(5, 0, 0),
                new PIDController(5, 0, 0),
                new PIDController(2, 0, 0))
            .withDefaultShouldFlip()
            .withPoseReset(drive::setPose);

    AutoBuilder.configure(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
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

  /**
   * Sets the velocity from ChassisSpeeds, determines which velocity method to run depending on the
   * robot mode.
   *
   * @param speeds ChassisSpeeds
   */
  public static void runVelocity(ChassisSpeeds speeds) {
    switch (Constants.Robot.robotMode) {
      case SHOOT:
        drive.runVelocityOffset(
            new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                (trackingSupplier != null)
                    ? getOmega(trackingSupplier)
                    : speeds.omegaRadiansPerSecond),
            Constants.ShooterConstants.location);
        break;
      default:
        drive.runVelocity(
            new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                (trackingSupplier != null)
                    ? getOmega(trackingSupplier)
                    : speeds.omegaRadiansPerSecond));
    }
  }

  public static double getOmega(Supplier<Rotation2d> rotationSupplier) {
    Constants.Drive.ANGLE_PID.enableContinuousInput(-Math.PI, Math.PI);

    Logger.recordOutput("PathBuilder/Track Target Angle", rotationSupplier.get().getRadians());
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
  }

  /**
   * @return The robot chassis speeds, determined by the current robot mode, offsetted.
   */
  public static ChassisSpeeds getChassisSpeeds() {
    switch (Constants.Robot.robotMode) {
      case SHOOT:
        return drive.getChassisSpeedsOffset(Constants.ShooterConstants.location);
      default:
        return drive.getChassisSpeeds();
    }
  }

  /**
   * @return The robot position, determinedby the current robot mode, offsetted.
   */
  public static Pose2d getPose() {
    switch (Constants.Robot.robotMode) {
      case SHOOT:
        return drive.getPoseOffset(Constants.ShooterConstants.location);
      default:
        return drive.getPose();
    }
  }

  /**
   * A method that takes in a Transltion2d that the robot will orient towards.
   *
   * @param wanted A Translation2d Supplier
   */
  public static void targetTranslation(Supplier<Translation2d> wanted) {
    targetRotation(() -> wanted.get().minus(drive.getPose().getTranslation()).getAngle());

    trackingSupplier = () -> wanted.get().minus(drive.getPose().getTranslation()).getAngle();
  }

  /**
   * A method takes in a Rotation2d that the robot will orient towards
   *
   * @param wanted A Rotation2d Supplier
   */
  public static void targetRotation(Supplier<Rotation2d> wanted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    trackingSupplier = wanted;

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

  /*
   * Stops any current rotation tracking
   */
  public static void stopTarget() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    trackingSupplier = null;
  }

  /**
   * @return get current PathConstraints for AutoBuilder
   */
  public static PathConstraints getConstraints() {
    return constraints;
  }

  /**
   * @param newConstraints the new PathConstraints for AutoBuilder
   */
  public static void setConstraints(PathConstraints newConstraints) {
    constraints = newConstraints;
  }

  /**
   * Follows a path off of several Pose2d waypoints
   *
   * @param poses Pose2d
   * @return Command
   */
  public static Command followPath(Pose2d... poses) {
    return AutoBuilder.followPath(
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(poses),
            PathBuilder.getConstraints(),
            null,
            new GoalEndState(0, Rotation2d.kZero)));
  }

  public static Command followPath(List<Waypoint> waypoints) {
    return AutoBuilder.followPath(
        new PathPlannerPath(
            waypoints, PathBuilder.getConstraints(), null, new GoalEndState(0, Rotation2d.kZero)));
  }

  public static Command interpolatePath(Pose2d... poses) {
    if (poses.length < 2) {
      return Commands.none();
    }

    List<Waypoint> waypoints = new ArrayList<>();
    Translation2d prevTangent = null;

    for (int i = 0; i < poses.length; i++) {
      Translation2d anchor = poses[i].getTranslation();
      Translation2d nextAnchor = (i == poses.length - 1) ? null : poses[i + 1].getTranslation();
      Translation2d geomectricTangent =
          (nextAnchor != null) ? nextAnchor.minus(anchor) : prevTangent;

      // basically just uses the past and the future to find an spline translation
      if (geomectricTangent == null || geomectricTangent.getNorm() < 0.000001)
        geomectricTangent =
            new Translation2d(1.0, poses[i].getRotation()); // RAHH TRANSLATION2D HAS POLAR
      else geomectricTangent = geomectricTangent.div(geomectricTangent.getNorm());

      // combine incoming and outgoing tangents, making it one smooth path,
      // if there's a past, use the past, if not, use geo
      Translation2d tangent =
          (prevTangent == null)
              ? geomectricTangent
              : prevTangent
                  .plus(geomectricTangent)
                  .div(2.0)
                  .div(prevTangent.plus(geomectricTangent).getNorm());

      double distance =
          (nextAnchor != null)
              ? anchor.getDistance(nextAnchor) * 0.5
              : anchor.getDistance(poses[i - 1].getTranslation()) * 0.5;

      // Finds the translation2d for the controls
      Translation2d prevControl =
          (prevTangent == null) ? null : anchor.minus(prevTangent.times(distance));
      Translation2d nextControl =
          (nextAnchor == null) ? null : anchor.plus(tangent.times(distance));
      waypoints.add(new Waypoint(prevControl, anchor, nextControl));
      prevTangent = tangent;
    }

    return AutoBuilder.followPath(
        new PathPlannerPath(
            waypoints,
            PathBuilder.getConstraints(),
            null,
            new GoalEndState(
                0.0,
                poses[poses.length - 1]
                    .getRotation()))); // use real values instead of arbitrary values
  }

  // My own auto triggers :) very simple Commands but maintains a uniform structure through the
  // syntax

  public static Command triggerWhenClose(
      Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(
            () -> drive.getPose().getTranslation().getDistance(location) <= distance)
        .andThen(runnable);
  }

  public static Command triggerWhenFar(Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(
            () -> drive.getPose().getTranslation().getDistance(location) > distance)
        .andThen(runnable);
  }

  public static Command triggerWhenTrue(BooleanSupplier condition, Command runnable) {
    return Commands.waitUntil(condition).andThen(runnable);
  }

  public static Command triggerWithDelay(double seconds, Command runnable) {
    return Commands.waitSeconds(seconds).andThen(runnable);
  }

  private static Command shootTemp = Commands.print("Just a ShootCommand placeholder");
  private static Command intakeTemp = Commands.print("Just a IntakeCommand placeholder");
  private static Command climbTemp = Commands.print("Just a ClimbCommand placeholder");
  private static Command idleTemp = Commands.print("Just a IdleCommand placeholder");

  public static Command generalAuton(Pose2d... poses) {
    return Commands.parallel(
        PathBuilder.followPath(poses), // Path Command
        Commands.repeatingSequence(
                Commands.waitUntil(
                        () ->
                            drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(
                                            FieldConstants.Trench.left_trench_alliance_entrance)
                                    > Constants.Robot.PATH_ERROR
                                && drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(
                                            FieldConstants.Trench.right_trench_alliance_entrance)
                                    > Constants.Robot.PATH_ERROR)
                    .andThen(
                        Commands.runOnce(() -> Constants.Robot.robotMode = RobotMode.SHOOT)
                            .andThen(
                                shootTemp
                                    .alongWith(
                                        Commands.runOnce(
                                            () ->
                                                PathBuilder.targetRotation(
                                                    () ->
                                                        new Rotation2d(
                                                            ProjMath.movingShot(
                                                                    20, // placeholder
                                                                    FieldConstants.Hub.hub_center,
                                                                    new Translation2d(
                                                                        drive.getChassisSpeeds()
                                                                            .vxMetersPerSecond,
                                                                        drive.getChassisSpeeds()
                                                                            .vyMetersPerSecond))
                                                                .getZ()))))
                                    .until(
                                        () ->
                                            drive
                                                        .getPose()
                                                        .getTranslation()
                                                        .getDistance(
                                                            FieldConstants.Trench
                                                                .left_trench_alliance_entrance)
                                                    <= Constants.Robot.PATH_ERROR
                                                || drive
                                                        .getPose()
                                                        .getTranslation()
                                                        .getDistance(
                                                            FieldConstants.Trench
                                                                .right_trench_alliance_entrance)
                                                    <= Constants.Robot.PATH_ERROR)
                                    .andThen(
                                        Commands.parallel(
                                            Commands.print("Just a IdleCommand placeholder"),
                                            Commands.runOnce(() -> PathBuilder.stopTarget()))))),
                Commands.waitUntil(
                        () ->
                            drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(
                                            FieldConstants.Trench.left_trench_neutral_entrance)
                                    > Constants.Robot.PATH_ERROR
                                && drive
                                        .getPose()
                                        .getTranslation()
                                        .getDistance(
                                            FieldConstants.Trench.right_trench_neutral_entrance)
                                    > Constants.Robot.PATH_ERROR)
                    .andThen(
                        Commands.runOnce(() -> Constants.Robot.robotMode = RobotMode.INTAKE)
                            .andThen(
                                intakeTemp
                                    .until(
                                        () ->
                                            drive
                                                        .getPose()
                                                        .getTranslation()
                                                        .getDistance(
                                                            FieldConstants.Trench
                                                                .left_trench_neutral_entrance)
                                                    <= Constants.Robot.PATH_ERROR
                                                || drive
                                                        .getPose()
                                                        .getTranslation()
                                                        .getDistance(
                                                            FieldConstants.Trench
                                                                .right_trench_neutral_entrance)
                                                    <= Constants.Robot.PATH_ERROR)
                                    .andThen(idleTemp))))
            .andThen(climbTemp));
  }

  /*
   *  How the PathBuilder Auton Structure should look like
   *  PathBuilder.generalAuton(Pose2d... poses));
   *
   *
   *
   */

  /**
   * Creates a path off of a goal Pose2d using Pathfinding
   *
   * @param endPose Pose2d
   * @return Command
   */
  public static Command createPath(Pose2d endPose) {
    return AutoBuilder.pathfindToPose(endPose, constraints, 0.0).finallyDo(() -> drive.stop());
  }

  /**
   * Creates a path off of several Pose2d using Pathfinding
   *
   * @param poses several Pose2d
   * @return Command
   */
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

  /**
   * Creates a path off of several Translation2d using Pathfinding
   *
   * @param translations several Translation2d
   * @return Command
   */
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

  /**
   * Creates a path off of a goal Translation2d using Pathfinding
   *
   * @param endTranslation Translation2d
   * @return Command
   */
  public static Command createPath(Translation2d endTranslation) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(endTranslation, new Rotation2d()), constraints, 0.0)
        .beforeStarting(() -> Constants.Drive.ANGLE_PID.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }

  /**
   * Creates a path off of a goal Pose2d and an end velocity using Pathfinding
   *
   * @param endPose Pose2d
   * @param endVel double
   * @return Command
   */
  public static Command createPath(Pose2d endPose, double endVel) {
    return AutoBuilder.pathfindToPose(endPose, constraints, endVel).finallyDo(() -> drive.stop());
  }

  /**
   * Creates a path off of a goal Translation2d and an end velocity using Pathfinding
   *
   * @param endTranslation Translation2d
   * @param endVel double
   * @return Command
   */
  public static Command createPath(Translation2d endTranslation, double endVel) {
    return AutoBuilder.pathfindToPose(
            new Pose2d(endTranslation, new Rotation2d()), constraints, endVel)
        .beforeStarting(() -> Constants.Drive.ANGLE_PID.reset(drive.getRotation().getRadians()));
    // .finallyDo(() -> drive.stop());
  }
  // TODO: uncomment this above line if you want priyanshu

  /**
   * Merges into a path finding using Pathfinding
   *
   * @param knownPath PathPlannerPath
   * @return Command
   */
  public static Command mergeToPath(PathPlannerPath knownPath) {
    return AutoBuilder.pathfindThenFollowPath(knownPath, constraints)
        .beforeStarting(() -> Constants.Drive.ANGLE_PID.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> drive.stop());
  }
}
