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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.lib.BLine.*;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.subsystems.drive.Drive;
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
          Constants.Drive.DRIVE_MAXVEL * 0.8,
          Constants.Drive.DRIVE_MAXACC * 0.8,
          Constants.Drive.ANGLE_MAXVEL * 0.8,
          Constants.Drive.ANGLE_MAXACC * 0.8);

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
                new PIDController(
                    Constants.Drive.DRIVE_PID.getP(),
                    Constants.Drive.DRIVE_PID.getI(),
                    Constants.Drive.DRIVE_PID.getD()),
                new PIDController(
                    Constants.Drive.ANGLE_PID.getP(),
                    Constants.Drive.ANGLE_PID.getI(),
                    Constants.Drive.ANGLE_PID.getD()),
                new PIDController(2, 0, 0))
            .withDefaultShouldFlip()
            .withPoseReset(drive::setPose);

    // Set global constraints before creating any paths
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            PathBuilder.getConstraints().maxVelocityMPS(),
            PathBuilder.getConstraints().maxAccelerationMPSSq(),
            Units.radiansToDegrees(PathBuilder.getConstraints().maxAngularVelocityRadPerSec()),
            Units.radiansToDegrees(
                PathBuilder.getConstraints().maxAngularAccelerationRadPerSecSq()),
            Constants.Drive.BLINE_DRIVE_TOL,
            Constants.Drive.BLINE_ROT_TOL.magnitude(),
            Constants.Drive.BLINE_HANDOFF_RADIUS));

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
                    ? drive.getOmega(trackingSupplier)
                    : speeds.omegaRadiansPerSecond),
            new Translation2d());
        break;
      default:
        drive.runVelocity(
            new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                (trackingSupplier != null)
                    ? drive.getOmega(trackingSupplier)
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
        return drive.getChassisSpeedsOffset(new Translation2d());
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
        return drive.getPoseOffset(new Translation2d());
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

  // public static Command followPolylinePath(Pose2d firstPose, PathElement... elements) {
  //   List<PathElement> list = List.of(elements);
  //   list.add(0, new Path.Waypoint(firstPose));
  //   Path bLinePath = new Path(list);

  //   return Commands.sequence(
  //       PathBuilder.createPath(firstPose),
  //       Commands.runOnce(() ->
  // drive.setModuleOrientations(bLinePath.getInitialModuleDirection())),
  //       pathBuilder.build(bLinePath));
  // }

  public static Command followPolylinePath(PathElement... elements) {
    List<PathElement> list = new ArrayList<PathElement>();
    drive.setPose(new Pose2d(new Translation2d(3.54, 2), Rotation2d.kZero));
    list.add(new Path.Waypoint(PathBuilder.getPose()));
    for (PathElement el : elements) list.add(el);
    Path bLinePath = new Path(list);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setModuleOrientations(bLinePath.getInitialModuleDirection())),
        pathBuilder.build(bLinePath));
  }

  /**
   * Follows a path off of several Pose2d waypoints
   *
   * @param poses Pose2d
   * @return Command
   */
  public static Command followTimedPath(Pose2d... poses) {
    return AutoBuilder.followPath(
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(poses),
            PathBuilder.getConstraints(),
            null,
            new GoalEndState(0, Rotation2d.kZero)));
  }

  public static Command followTimedPath(List<Waypoint> waypoints) {
    return AutoBuilder.followPath(
        new PathPlannerPath(
            waypoints, PathBuilder.getConstraints(), null, new GoalEndState(0, Rotation2d.kZero)));
  }

  public static Command interpolateTimedPath(Pose2d... poses) {
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

    getPathTime(
        new PathPlannerPath(
            waypoints,
            PathBuilder.getConstraints(),
            null,
            new GoalEndState(0.0, poses[poses.length - 1].getRotation())));

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

  public static double getPathTime(PathPlannerPath path) {
    Logger.recordOutput(
        "Auton/Trajectory Time",
        path.getIdealTrajectory(Constants.Drive.PP_CONFIG).orElse(null).getTotalTimeSeconds());
    return path.getIdealTrajectory(Constants.Drive.PP_CONFIG).orElse(null).getTotalTimeSeconds();
  }

  public static Command interpolateTimedPath(List<Pose2d> poses_) {
    Pose2d[] poses = poses_.toArray(new Pose2d[0]);
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
    return Commands.waitUntil(() -> getPose().getTranslation().getDistance(location) <= distance)
        .andThen(runnable);
  }

  public static Command triggerWhenFar(Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(() -> getPose().getTranslation().getDistance(location) > distance)
        .andThen(runnable);
  }

  public static Command triggerWhenTrue(BooleanSupplier condition, Command runnable) {
    return Commands.waitUntil(condition).andThen(runnable);
  }

  public static Command triggerWithDelay(double seconds, Command runnable) {
    return Commands.waitSeconds(seconds).andThen(runnable);
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
