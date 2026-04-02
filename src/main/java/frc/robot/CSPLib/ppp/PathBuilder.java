package frc.robot.CSPLib.ppp;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A CSP-made class utilizing PathPlanner AutoBuilder, allowing us to create paths, follow paths,
 * and get drive properties based on the robot mode.
 */
public final class PathBuilder {
  private static final double ROTATION_POSITION_EPS = 1e-6;

  private static Drive drive;

  // prob too fast
  private static PathConstraints constraints =
      new PathConstraints(
          Constants.DriveConstants.DRIVE_MAXVEL * 0.8,
          Constants.DriveConstants.DRIVE_MAXACC * 0.8,
          Constants.DriveConstants.ANGLE_MAXVEL * 0.8,
          Constants.DriveConstants.ANGLE_MAXACC * 0.8);

  private static double ROTATION_TOL_RAD = Math.toRadians(1);
  private static double FOLLOW_ROTATION_SAMPLE_METERS = 0.5;
  private static double PATH_CREATION_TOL = Units.inchesToMeters(1);

  private static boolean logged;
  private static double angleFF;
  private static Rotation2d angleTol;
  private static Supplier<Pose2d> getPose;
  private static Supplier<ChassisSpeeds> getChassisSpeeds;
  private static ProfiledPIDController driveController;
  private static ProfiledPIDController angleController;
  private static Runnable stopDrive;

  private static List<Pose2d> activeFollowSampled = Collections.emptyList();
  private static double[] activeFollowCum = new double[0];
  private static double activeFollowTotalLength = 0.0;
  private static double activeFollowWaypointSlots = 1.0;
  private static List<RotationSample> activeExplicitRotationSamples = Collections.emptyList();
  private static List<FollowWindow> activeFollowWindows = Collections.emptyList();
  private static boolean activeFollowEnabled = false;

  private static final class RotationSample {
    final double position;
    final Rotation2d rotation;

    RotationSample(double position, Rotation2d rotation) {
      this.position = position;
      this.rotation = rotation;
    }
  }

  private static final class FollowWindow {
    final double startArc;
    final double endArc;
    final double lookaheadMeters;
    final double spread;
    final double tangentWeight;
    final double sampleMeters;

    FollowWindow(
        double startArc,
        double endArc,
        double lookaheadMeters,
        double spread,
        double tangentWeight,
        double sampleMeters) {
      this.startArc = startArc;
      this.endArc = endArc;
      this.lookaheadMeters = lookaheadMeters;
      this.spread = spread;
      this.tangentWeight = tangentWeight;
      this.sampleMeters = sampleMeters;
    }
  }

  private static final class FollowProfile {
    final double leadMeters;
    final double spread;
    final double sampleMeters;
    final double tangentWeight;

    FollowProfile(double leadMeters, double spread, double sampleMeters, double tangentWeight) {
      this.leadMeters = leadMeters;
      this.spread = spread;
      this.sampleMeters = sampleMeters;
      this.tangentWeight = tangentWeight;
    }
  }

  public static void configurePathing(
      double pathCreationTolMeters, double rotationTolRadians, double followRotationMeters) {
    ROTATION_TOL_RAD = rotationTolRadians;
    PATH_CREATION_TOL = pathCreationTolMeters;
    FOLLOW_ROTATION_SAMPLE_METERS = followRotationMeters;
  }

  /**
   * A method to configure the PathBuilder class, setting it up with the Drivetrain instance.
   * Enables PathFinding and AutoBuilder.
   *
   * @param drivetrain
   */
  public static void configureDrive(
      boolean logged_,
      double angleFF_,
      Rotation2d angleTol_,
      Supplier<Pose2d> getPose_,
      Consumer<Pose2d> setPose_,
      Supplier<ChassisSpeeds> getChassisSpeeds_,
      Runnable stopDrive_,
      Consumer<ChassisSpeeds> runVelocity_,
      ProfiledPIDController driveController_,
      ProfiledPIDController angleController_,
      RobotConfig robotConfig_,
      BooleanSupplier shouldFlip_,
      Drive drive_) {

    stopDrive = stopDrive_;
    logged = logged_;
    getPose = getPose_;
    getChassisSpeeds = getChassisSpeeds_;
    driveController = driveController_;
    angleController = angleController_;
    drive = drive_;
    angleFF = angleFF_;
    angleTol = angleTol_;

    AutoBuilder.configure(
        getPose_,
        setPose_,
        getChassisSpeeds_,
        runVelocity_,
        new PPHolonomicDriveController(
            new PIDConstants(
                driveController.getP(), driveController.getI(), driveController.getD()),
            new PIDConstants(
                driveController.getP(), driveController.getI(), driveController.getD())),
        robotConfig_,
        shouldFlip_,
        drive_);

    if (logged) {
      PathPlannerLogging.setLogActivePathCallback(
          (activePath) ->
              Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));

      PathPlannerLogging.setLogTargetPoseCallback(
          (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));
    }
  }

  /**
   * A method that takes in a Translation2d that the robot will orient towards.
   *
   * @param wanted A Translation2d Supplier
   */
  public static void targetTranslation(Supplier<Translation2d> wanted) {
    targetRotation(() -> wanted.get().minus(getPose.get().getTranslation()).getAngle());
  }

  /**
   * A method takes in a Rotation2d that the robot will orient towards
   *
   * @param wanted A Rotation2d Supplier
   */
  public static void targetRotation(Supplier<Rotation2d> wanted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          Supplier<Rotation2d> rotationSupplier = wanted;
          angleController.enableContinuousInput(-Math.PI, Math.PI);

          if (logged) {
            Logger.recordOutput(
                "PathBuilder/Track Target Angle", rotationSupplier.get().getRadians());
            Logger.recordOutput(
                "PathBuilder/Track Current Angle", getPose.get().getRotation().getRadians());
          }

          double omega =
              angleController.calculate(
                      getPose.get().getRotation().getRadians(), rotationSupplier.get().getRadians())
                  + angleController.getSetpoint().velocity * angleFF;

          if (Math.abs(
                      getPose.get().getRotation().getRadians()
                          - rotationSupplier.get().getRadians())
                  < angleTol.getRadians()
              && angleController.getSetpoint().velocity == 0.0) {
            omega = 0.0;
          }

          return omega;
        });
  }

  /** Stops any current rotation tracking. */
  public static void stopTarget() {
    clearFollowRotationOverride();
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

  public static class Target {
    public enum RotationMode {
      LINEAR,
      SNAP,
      HOLD,
      FOLLOW
    }

    public final Pose2d pose;
    public final double speedMultiplier;
    public final double rotationSpeedMultiplier;
    public final double rotationLeadMeters;
    public final double rotationSpread;
    public final RotationMode rotationMode;
    public final Rotation2d heading;
    public final double tangentWeight;
    public final double curveScale;
    public final RotationTarget[] overrideRotations;
    public final Rotation2d startingRotation;
    public final Rotation2d endingRotation;
    public final Double controlDistanceBeforeMeters;
    public final Double controlDistanceAfterMeters;
    public final Double startingSpeedMetersPerSecond;
    public final Double endingSpeedMetersPerSecond;
    public final BooleanSupplier condition;
    public final double toleranceMeters;
    public final Supplier<Command> command;

    public Target(Pose2d pose) {
      this(
          pose,
          1.0,
          1.0,
          0.0,
          1.0,
          RotationMode.LINEAR,
          null,
          1.0,
          1.0,
          null, // overrideRotations
          null, // startingRotation
          null, // endingRotation
          null, // controlDistanceBeforeMeters
          null, // controlDistanceAfterMeters
          null, // startingSpeedMetersPerSecond
          null, // endingSpeedMetersPerSecond
          PATH_CREATION_TOL,
          null,
          () -> true);
    }

    public Target(Pose2d pose, double speedMultiplier) {
      this(
          pose,
          speedMultiplier,
          1.0,
          0.0,
          1.0,
          RotationMode.LINEAR,
          null,
          1.0,
          1.0,
          null,
          null,
          null,
          null,
          null,
          null,
          null,
          PATH_CREATION_TOL,
          null,
          () -> true);
    }

    public Target(Pose2d pose, double speedMultiplier, double rotationLeadMeters) {
      this(
          pose,
          speedMultiplier,
          1.0,
          rotationLeadMeters,
          1.0,
          RotationMode.LINEAR,
          null,
          1.0,
          1.0,
          null,
          null,
          null,
          null,
          null,
          null,
          null,
          PATH_CREATION_TOL,
          null,
          () -> true);
    }

    public Target(
        Pose2d pose, double speedMultiplier, double rotationLeadMeters, double rotationSpread) {
      this(
          pose,
          speedMultiplier,
          1.0,
          rotationLeadMeters,
          rotationSpread,
          RotationMode.LINEAR,
          null,
          1.0,
          1.0,
          null,
          null,
          null,
          null,
          null,
          null,
          null,
          PATH_CREATION_TOL,
          null,
          () -> true);
    }

    private Target(
        Pose2d pose,
        double speedMultiplier,
        double rotationSpeedMultiplier,
        double rotationLeadMeters,
        double rotationSpread,
        RotationMode rotationMode,
        Rotation2d heading,
        double tangentWeight,
        double curveScale,
        RotationTarget[] overrideRotations,
        Rotation2d startingRotation,
        Rotation2d endingRotation,
        Double controlDistanceBeforeMeters,
        Double controlDistanceAfterMeters,
        Double startingSpeedMetersPerSecond,
        Double endingSpeedMetersPerSecond,
        double toleranceMeters,
        Supplier<Command> command,
        BooleanSupplier condition) {

      if (pose == null) {
        throw new IllegalArgumentException("pose cannot be null");
      }
      if (!Double.isFinite(speedMultiplier) || speedMultiplier <= 0.0) {
        throw new IllegalArgumentException("speedMultiplier must be finite and > 0");
      }
      if (!Double.isFinite(rotationSpeedMultiplier) || rotationSpeedMultiplier <= 0.0) {
        throw new IllegalArgumentException("rotationSpeedMultiplier must be finite and > 0");
      }
      if (!Double.isFinite(rotationLeadMeters)) {
        throw new IllegalArgumentException("rotationLeadMeters must be finite");
      }
      if (!Double.isFinite(rotationSpread) || rotationSpread <= 0.0) {
        throw new IllegalArgumentException("rotationSpread must be finite and > 0");
      }
      if (!Double.isFinite(tangentWeight) || tangentWeight < 0.0) {
        throw new IllegalArgumentException("tangentWeight must be finite and >= 0");
      }
      if (!Double.isFinite(curveScale) || curveScale < 0.0 || curveScale > 1.0) {
        throw new IllegalArgumentException("curveScale must be finite and between 0 and 1");
      }
      if (overrideRotations != null) {
        for (RotationTarget target : overrideRotations) {
          if (target == null) {
            throw new IllegalArgumentException("overrideRotations cannot contain null");
          }
        }
      }
      if (controlDistanceBeforeMeters != null
          && (!Double.isFinite(controlDistanceBeforeMeters) || controlDistanceBeforeMeters < 0.0)) {
        throw new IllegalArgumentException("controlDistanceBeforeMeters must be finite and >= 0");
      }
      if (controlDistanceAfterMeters != null
          && (!Double.isFinite(controlDistanceAfterMeters) || controlDistanceAfterMeters < 0.0)) {
        throw new IllegalArgumentException("controlDistanceAfterMeters must be finite and >= 0");
      }
      if (startingSpeedMetersPerSecond != null
          && (!Double.isFinite(startingSpeedMetersPerSecond)
              || startingSpeedMetersPerSecond < 0.0)) {
        throw new IllegalArgumentException("startingSpeedMetersPerSecond must be finite and >= 0");
      }
      if (endingSpeedMetersPerSecond != null
          && (!Double.isFinite(endingSpeedMetersPerSecond) || endingSpeedMetersPerSecond < 0.0)) {
        throw new IllegalArgumentException("endingSpeedMetersPerSecond must be finite and >= 0");
      }
      if (!Double.isFinite(toleranceMeters) || toleranceMeters < 0.0) {
        throw new IllegalArgumentException("toleranceMeters must be finite and >= 0");
      }

      this.pose = pose;
      this.speedMultiplier = speedMultiplier;
      this.rotationSpeedMultiplier = rotationSpeedMultiplier;
      this.rotationLeadMeters = rotationLeadMeters;
      this.rotationSpread = rotationSpread;
      this.rotationMode = rotationMode == null ? RotationMode.LINEAR : rotationMode;
      this.heading = heading;
      this.tangentWeight = tangentWeight;
      this.curveScale = curveScale;
      this.overrideRotations = overrideRotations == null ? null : overrideRotations.clone();
      this.startingRotation = startingRotation;
      this.endingRotation = endingRotation;
      this.controlDistanceBeforeMeters = controlDistanceBeforeMeters;
      this.controlDistanceAfterMeters = controlDistanceAfterMeters;
      this.startingSpeedMetersPerSecond = startingSpeedMetersPerSecond;
      this.endingSpeedMetersPerSecond = endingSpeedMetersPerSecond;
      this.toleranceMeters = toleranceMeters;
      this.command = command;
      this.condition = condition == null ? () -> true : condition;
    }

    public Target withSpeed(double speedMultiplier) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withRotationSpeed(double rotationSpeedMultiplier) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withRotationLead(double rotationLeadMeters) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withRotationSpread(double rotationSpread) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withRotationMode(RotationMode rotationMode) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withHeading(Rotation2d heading) {
      if (heading == null) {
        throw new IllegalArgumentException("heading cannot be null");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withControlDistances(double beforeMeters, double afterMeters) {
      if (!Double.isFinite(beforeMeters) || beforeMeters < 0.0) {
        throw new IllegalArgumentException("beforeMeters must be finite and >= 0");
      }
      if (!Double.isFinite(afterMeters) || afterMeters < 0.0) {
        throw new IllegalArgumentException("afterMeters must be finite and >= 0");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          beforeMeters,
          afterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withStartingSpeed(double startingSpeedMetersPerSecond) {
      if (!Double.isFinite(startingSpeedMetersPerSecond) || startingSpeedMetersPerSecond < 0.0) {
        throw new IllegalArgumentException("startingSpeedMetersPerSecond must be finite and >= 0");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withEndingSpeed(double endingSpeedMetersPerSecond) {
      if (!Double.isFinite(endingSpeedMetersPerSecond) || endingSpeedMetersPerSecond < 0.0) {
        throw new IllegalArgumentException("endingSpeedMetersPerSecond must be finite and >= 0");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withOverrideRotations(RotationTarget... overrideRotations) {
      if (overrideRotations == null) {
        throw new IllegalArgumentException("overrideRotations cannot be null");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withStartingRotation(Rotation2d startingRotation) {
      if (startingRotation == null) {
        throw new IllegalArgumentException("startingRotation cannot be null");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withEndingRotation(Rotation2d endingRotation) {
      if (endingRotation == null) {
        throw new IllegalArgumentException("endingRotation cannot be null");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withTangentWeight(double tangentWeight) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withCurve(double curveScale) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target onlyIf(BooleanSupplier condition) {
      if (condition == null) {
        throw new IllegalArgumentException("condition cannot be null");
      }

      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withTolerance(double toleranceMeters) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }

    public Target withCommand(Command command) {
      return withCommand(() -> command);
    }

    public Target withCommand(Supplier<Command> command) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
          curveScale,
          overrideRotations,
          startingRotation,
          endingRotation,
          controlDistanceBeforeMeters,
          controlDistanceAfterMeters,
          startingSpeedMetersPerSecond,
          endingSpeedMetersPerSecond,
          toleranceMeters,
          command,
          condition);
    }
  }

  // Way too much work into this
  public static Command path(Target... targets) {
    if (targets == null || targets.length == 0) {
      throw new IllegalArgumentException("Must supply at least one Target");
    }

    final double rotIndexRadius = Constants.DriveConstants.PATH_CREATION_TOL;

    List<Target> activeTargets = new ArrayList<>();
    for (Target target : targets) {
      if (target == null) {
        throw new IllegalArgumentException("targets cannot contain null");
      }
      if (target.condition.getAsBoolean()) {
        activeTargets.add(target);
      }
    }

    if (activeTargets.isEmpty()) {
      return Commands.none();
    }

    Pose2d[] poses = new Pose2d[activeTargets.size()];
    for (int i = 0; i < activeTargets.size(); i++) {
      poses[i] = activeTargets.get(i).pose;
    }

    List<Pose2d> travelHeadingPoses = new ArrayList<>(poses.length);
    for (int i = 0; i < poses.length; i++) {
      Translation2d pos = poses[i].getTranslation();
      Target target = activeTargets.get(i);

      Rotation2d splineHeading;
      Rotation2d requestedHeading = target.heading;

      if (requestedHeading != null) {
        splineHeading = requestedHeading;
      } else if (poses.length == 1) {
        splineHeading = poses[i].getRotation();
      } else if (i == 0) {
        Translation2d next = poses[i + 1].getTranslation();
        splineHeading =
            safeHeading(next.getX() - pos.getX(), next.getY() - pos.getY(), poses[i].getRotation());
      } else if (i == poses.length - 1) {
        Translation2d prev = poses[i - 1].getTranslation();
        splineHeading =
            safeHeading(pos.getX() - prev.getX(), pos.getY() - prev.getY(), poses[i].getRotation());
      } else {
        Translation2d prev = poses[i - 1].getTranslation();
        Translation2d next = poses[i + 1].getTranslation();
        splineHeading =
            safeHeading(
                next.getX() - prev.getX(), next.getY() - prev.getY(), poses[i].getRotation());
      }

      Rotation2d straightHeading;
      if (requestedHeading != null) {
        straightHeading = requestedHeading;
      } else if (poses.length == 1) {
        straightHeading = poses[i].getRotation();
      } else if (i == 0) {
        Translation2d next = poses[i + 1].getTranslation();
        straightHeading =
            safeHeading(next.getX() - pos.getX(), next.getY() - pos.getY(), poses[i].getRotation());
      } else if (i == poses.length - 1) {
        Translation2d prev = poses[i - 1].getTranslation();
        straightHeading =
            safeHeading(pos.getX() - prev.getX(), pos.getY() - prev.getY(), poses[i].getRotation());
      } else {
        Translation2d prev = poses[i - 1].getTranslation();
        Translation2d next = poses[i + 1].getTranslation();
        straightHeading =
            safeHeading(
                next.getX() - prev.getX(), next.getY() - prev.getY(), poses[i].getRotation());
      }

      double curveScale = clamp01(target.curveScale);
      Rotation2d finalHeading = interpolateRotation(straightHeading, splineHeading, curveScale);

      travelHeadingPoses.add(new Pose2d(pos, finalHeading));
    }

    List<Waypoint> waypoints = waypointsFromPosesWithCurve(travelHeadingPoses, activeTargets);
    final int waypointSlots = Math.max(1, waypoints.size() - 1);

    List<RotationTarget> emptyRotationTargets = Collections.<RotationTarget>emptyList();
    List<PointTowardsZone> emptyPointTowards = Collections.<PointTowardsZone>emptyList();
    List<ConstraintsZone> emptyConstraintsZones = Collections.<ConstraintsZone>emptyList();
    List<EventMarker> emptyEventMarkers = Collections.<EventMarker>emptyList();

    PathConstraints globalConstraints = getConstraints();

    Double startingSpeedMps = activeTargets.get(0).startingSpeedMetersPerSecond;
    Double endingSpeedMps = activeTargets.get(activeTargets.size() - 1).endingSpeedMetersPerSecond;

    Target overrideSource = getFirstOverrideTarget(activeTargets);

    Rotation2d startRotationForPath =
        overrideSource != null && overrideSource.startingRotation != null
            ? overrideSource.startingRotation
            : poses[0].getRotation();

    Rotation2d endRotationForPath =
        overrideSource != null && overrideSource.endingRotation != null
            ? overrideSource.endingRotation
            : poses[poses.length - 1].getRotation();

    IdealStartingState idealStartingState =
        startingSpeedMps != null
            ? new IdealStartingState(startingSpeedMps, startRotationForPath)
            : null;

    GoalEndState tmpGoal = new GoalEndState(0.0, endRotationForPath);
    PathPlannerPath tempPath =
        new PathPlannerPath(
            waypoints,
            emptyRotationTargets,
            emptyPointTowards,
            emptyConstraintsZones,
            emptyEventMarkers,
            globalConstraints,
            idealStartingState,
            tmpGoal,
            false);

    List<Pose2d> sampled = tempPath.getPathPoses();

    if (sampled == null || sampled.isEmpty()) {
      List<RotationTarget> fallbackRT = new ArrayList<>();
      List<EventMarker> fallbackMarkers = new ArrayList<>();
      int n = poses.length;

      if (overrideSource != null) {
        fallbackRT =
            buildOverrideRotationTargets(
                activeTargets, waypointSlots, startRotationForPath, endRotationForPath);
      } else {
        fallbackRT.add(new RotationTarget(0.0, poses[0].getRotation()));
        Rotation2d lastFallbackRotation = poses[0].getRotation();

        for (int i = 1; i < n; i++) {
          double pct = (n == 1) ? 0.0 : ((double) i) / (n - 1);
          Target target = activeTargets.get(i);

          if (target.command != null) {
            fallbackMarkers.add(
                new EventMarker(
                    "target_" + i,
                    pct * waypointSlots,
                    Commands.defer(target.command, Collections.emptySet())));
          }

          Rotation2d desired;
          switch (target.rotationMode) {
            case FOLLOW:
              desired = travelHeadingPoses.get(i).getRotation();
              break;
            case HOLD:
              desired = null;
              break;
            case SNAP:
            case LINEAR:
            default:
              desired = poses[i].getRotation();
              break;
          }

          if (desired != null
              && Math.abs(normalizeAngle(desired.getRadians() - lastFallbackRotation.getRadians()))
                  > Math.toRadians(0.1)) {
            double pos = pct * waypointSlots;

            if (target.rotationMode == Target.RotationMode.SNAP) {
              double eps = 1e-6;
              double pre = Math.max(0.0, pos - eps);
              double post = Math.min(waypointSlots, pos + eps);
              fallbackRT.add(new RotationTarget(pre, lastFallbackRotation));
              fallbackRT.add(new RotationTarget(post, desired));
            } else {
              fallbackRT.add(new RotationTarget(pos, desired));
            }

            lastFallbackRotation = desired;
          }
        }

        endRotationForPath = lastFallbackRotation;
      }

      Double fallbackEndingSpeed = endingSpeedMps != null ? endingSpeedMps : 0.0;
      GoalEndState finalGoal = new GoalEndState(fallbackEndingSpeed, endRotationForPath);
      PathPlannerPath fallback =
          new PathPlannerPath(
              waypoints,
              fallbackRT,
              emptyPointTowards,
              emptyConstraintsZones,
              fallbackMarkers.isEmpty() ? emptyEventMarkers : fallbackMarkers,
              globalConstraints,
              idealStartingState,
              finalGoal,
              false);

      clearFollowRotationOverride();
      return AutoBuilder.followPath(fallback);
    }

    int m = sampled.size();
    double[] cum = new double[m];
    cum[0] = 0.0;
    for (int i = 1; i < m; i++) {
      double seg = sampled.get(i).getTranslation().getDistance(sampled.get(i - 1).getTranslation());
      cum[i] = cum[i - 1] + seg;
    }

    double totalLength = cum[m - 1];
    if (totalLength <= 1e-9) {
      totalLength = 1e-9;
    }

    int searchSegStart = 0;
    double lastArcAccepted = 0.0;
    int nTargets = activeTargets.size();
    double[] targetArcs = new double[nTargets];

    for (int t = 0; t < nTargets; t++) {
      Target target = activeTargets.get(t);
      Translation2d targetTrans = target.pose.getTranslation();
      double snapTol =
          Double.isFinite(target.toleranceMeters) ? target.toleranceMeters : rotIndexRadius;

      if (sampled.size() == 1) {
        double d = sampled.get(0).getTranslation().getDistance(targetTrans);
        targetArcs[t] = (d <= snapTol) ? 0.0 : totalLength;
        if (targetArcs[t] < lastArcAccepted) {
          targetArcs[t] = lastArcAccepted;
        }
        lastArcAccepted = targetArcs[t];
        continue;
      }

      double bestPointDist = Double.POSITIVE_INFINITY;
      int bestPointIdx = Math.min(searchSegStart, sampled.size() - 1);
      for (int i = searchSegStart; i < sampled.size(); i++) {
        double d = sampled.get(i).getTranslation().getDistance(targetTrans);
        if (d < bestPointDist) {
          bestPointDist = d;
          bestPointIdx = i;
        }
      }

      double bestDist = Double.POSITIVE_INFINITY;
      double bestArc = Double.POSITIVE_INFINITY;
      int bestSeg = Math.max(0, searchSegStart);

      if (bestPointDist <= snapTol) {
        bestArc = cum[bestPointIdx];
        bestDist = bestPointDist;
        bestSeg = Math.max(0, bestPointIdx - 1);
      } else {
        for (int i = searchSegStart; i < sampled.size() - 1; i++) {
          Translation2d a = sampled.get(i).getTranslation();
          Translation2d b = sampled.get(i + 1).getTranslation();
          double ax = a.getX(), ay = a.getY(), bx = b.getX(), by = b.getY();
          double dx = bx - ax, dy = by - ay;
          double segLenSq = dx * dx + dy * dy;

          double u = 0.0;
          if (segLenSq > 1e-12) {
            double tx = targetTrans.getX() - ax;
            double ty = targetTrans.getY() - ay;
            u = (tx * dx + ty * dy) / segLenSq;
            if (u < 0.0) {
              u = 0.0;
            } else if (u > 1.0) {
              u = 1.0;
            }
          }

          double projX = ax + u * dx;
          double projY = ay + u * dy;
          double dist = Math.hypot(targetTrans.getX() - projX, targetTrans.getY() - projY);

          double segLen = Math.hypot(dx, dy);
          double arcAlong = cum[i] + (segLen * u);

          if (arcAlong + 1e-9 < lastArcAccepted) {
            continue;
          }

          if (dist < bestDist) {
            bestDist = dist;
            bestArc = arcAlong;
            bestSeg = i;
          }
        }

        if (bestDist == Double.POSITIVE_INFINITY) {
          double bestSampleDist = Double.POSITIVE_INFINITY;
          int bestSampleIdx = Math.min(searchSegStart, sampled.size() - 1);
          for (int i = searchSegStart; i < sampled.size(); i++) {
            double d = sampled.get(i).getTranslation().getDistance(targetTrans);
            if (d < bestSampleDist) {
              bestSampleDist = d;
              bestSampleIdx = i;
            }
          }
          int segIdx = Math.max(0, Math.min(sampled.size() - 2, bestSampleIdx - 1));
          Translation2d a = sampled.get(segIdx).getTranslation();
          Translation2d b = sampled.get(segIdx + 1).getTranslation();
          double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
          double segLenSq = dx * dx + dy * dy;
          double u = 0.0;
          if (segLenSq > 1e-12) {
            double tx = targetTrans.getX() - a.getX();
            double ty = targetTrans.getY() - a.getY();
            u = (tx * dx + ty * dy) / segLenSq;
            if (u < 0.0) {
              u = 0.0;
            } else if (u > 1.0) {
              u = 1.0;
            }
          }
          double segLen = Math.hypot(dx, dy);
          bestArc = cum[segIdx] + (segLen * u);
          bestSeg = segIdx;
        }
      }

      if (bestArc < lastArcAccepted) {
        bestArc = lastArcAccepted;
      }
      if (bestArc > totalLength) {
        bestArc = totalLength;
      }

      targetArcs[t] = bestArc;
      lastArcAccepted = bestArc;
      searchSegStart = Math.min(bestSeg, sampled.size() - 2);
    }

    List<RotationTarget> rotationTargets;
    List<RotationSample> rotationSamples = new ArrayList<>();
    List<FollowWindow> followWindows = new ArrayList<>();
    List<EventMarker> eventMarkers = new ArrayList<>();

    if (overrideSource != null) {
      rotationTargets =
          buildOverrideRotationTargets(
              activeTargets, waypointSlots, startRotationForPath, endRotationForPath);
      rotationSamples = rotationSamplesFromRotationTargets(rotationTargets);

      for (int t = 1; t < nTargets; t++) {
        Target target = activeTargets.get(t);
        double arc = targetArcs[t];
        double waypointPos = (arc / totalLength) * waypointSlots;

        if (target.command != null) {
          eventMarkers.add(
              new EventMarker(
                  "target_" + t,
                  waypointPos,
                  Commands.defer(target.command, Collections.emptySet())));
        }
      }

      activeFollowEnabled = false;
    } else {
      rotationTargets = new ArrayList<>();
      rotationTargets.add(new RotationTarget(0.0, poses[0].getRotation()));
      rotationSamples.add(new RotationSample(0.0, poses[0].getRotation()));
      Rotation2d lastRotation = poses[0].getRotation();
      final double PRE_OFFSET_ARC = Math.max(1e-3, totalLength * 1e-6);

      for (int t = 1; t < nTargets; t++) {
        Target target = activeTargets.get(t);
        double arc = targetArcs[t];
        double waypointPos = (arc / totalLength) * waypointSlots;

        if (target.command != null) {
          eventMarkers.add(
              new EventMarker(
                  "target_" + t,
                  waypointPos,
                  Commands.defer(target.command, Collections.emptySet())));
        }

        switch (target.rotationMode) {
          case FOLLOW:
            {
              FollowProfile profile = computeFollowProfile(sampled, cum, arc, totalLength, target);

              double activation = Math.max(profile.leadMeters, profile.sampleMeters);
              double startArc = Math.max(0.0, arc - activation);
              double endArc = Math.min(totalLength, arc + activation * profile.spread);

              followWindows.add(
                  new FollowWindow(
                      startArc,
                      endArc,
                      profile.leadMeters,
                      profile.spread,
                      profile.tangentWeight,
                      profile.sampleMeters));

              Rotation2d followHeading =
                  getBiasedTangentAtArc(sampled, cum, arc, totalLength, profile.sampleMeters);

              lastRotation =
                  emitRotationTarget(
                      rotationTargets,
                      arc,
                      followHeading,
                      lastRotation,
                      totalLength,
                      waypointSlots,
                      false);

              rotationSamples.add(new RotationSample(waypointPos, followHeading));
              break;
            }

          case HOLD:
            break;

          case SNAP:
          case LINEAR:
          default:
            {
              Rotation2d desired = poses[t].getRotation();

              double angDiff =
                  Math.abs(normalizeAngle(desired.getRadians() - lastRotation.getRadians()));
              if (target.rotationMode != Target.RotationMode.SNAP && angDiff <= ROTATION_TOL_RAD) {
                rotationSamples.add(new RotationSample(waypointPos, desired));
                break;
              }

              double leadMeters = target.rotationLeadMeters;
              double rotationStartArc = arc - leadMeters;
              rotationStartArc = Math.max(0.0, Math.min(rotationStartArc, totalLength));

              double baseEndArc = (leadMeters >= 0.0) ? arc : (arc - leadMeters);
              baseEndArc = Math.max(0.0, Math.min(baseEndArc, totalLength));

              double baseWindow = Math.abs(baseEndArc - rotationStartArc);
              if (baseWindow < 1e-12) {
                baseWindow = PRE_OFFSET_ARC;
              }

              double spread = target.rotationSpread;
              if (!Double.isFinite(spread) || spread <= 0.0) {
                spread = 1.0;
              }

              double spreadWindow =
                  (target.rotationMode == Target.RotationMode.SNAP)
                      ? PRE_OFFSET_ARC
                      : (baseWindow * spread);

              double rotationEndArc = rotationStartArc + spreadWindow;
              rotationEndArc = Math.max(0.0, Math.min(rotationEndArc, totalLength));

              double preArc = rotationStartArc;
              double preWaypointPos = (preArc / totalLength) * waypointSlots;
              double finalWaypointPos = (rotationEndArc / totalLength) * waypointSlots;

              if (finalWaypointPos <= preWaypointPos + 1e-12) {
                finalWaypointPos = Math.min(waypointSlots, preWaypointPos + 1e-6);
                preWaypointPos = Math.max(0.0, finalWaypointPos - 1e-6);
              }

              rotationTargets.add(new RotationTarget(preWaypointPos, lastRotation));
              rotationSamples.add(new RotationSample(preWaypointPos, lastRotation));

              rotationTargets.add(new RotationTarget(finalWaypointPos, desired));
              rotationSamples.add(new RotationSample(finalWaypointPos, desired));

              lastRotation = desired;
              break;
            }
        }
      }

      activeFollowEnabled = !followWindows.isEmpty();
    }

    List<ConstraintsZone> constraintsZones = new ArrayList<>();

    for (int i = 0; i < nTargets - 1; i++) {
      Target current = activeTargets.get(i);

      double startArc = targetArcs[i];
      double endArc = targetArcs[i + 1];

      if (endArc - startArc < 1e-6) {
        continue;
      }

      double startFrac = startArc / totalLength;
      double endFrac = endArc / totalLength;

      double startWaypointPos = startFrac * waypointSlots;
      double endWaypointPos = endFrac * waypointSlots;

      double linearMult = current.speedMultiplier;
      double angularMult = current.rotationSpeedMultiplier;

      if (Math.abs(linearMult - 1.0) < 1e-9 && Math.abs(angularMult - 1.0) < 1e-9) {
        continue;
      }

      PathConstraints scaled =
          new PathConstraints(
              globalConstraints.maxVelocityMPS() * linearMult,
              globalConstraints.maxAccelerationMPSSq() * linearMult,
              globalConstraints.maxAngularVelocityRadPerSec() * angularMult,
              globalConstraints.maxAngularAccelerationRadPerSecSq() * angularMult);

      constraintsZones.add(new ConstraintsZone(startWaypointPos, endWaypointPos, scaled));
    }

    constraintsZones = mergeConstraintsZones(constraintsZones);

    Rotation2d finalRotation =
        rotationTargets.isEmpty()
            ? poses[poses.length - 1].getRotation()
            : rotationTargets.get(rotationTargets.size() - 1).rotation();

    Double finalEndingSpeed = endingSpeedMps != null ? endingSpeedMps : 0.0;
    GoalEndState finalGoal = new GoalEndState(finalEndingSpeed, finalRotation);

    PathPlannerPath finalPath =
        new PathPlannerPath(
            waypoints,
            rotationTargets,
            emptyPointTowards,
            constraintsZones.isEmpty() ? emptyConstraintsZones : constraintsZones,
            eventMarkers.isEmpty() ? emptyEventMarkers : eventMarkers,
            globalConstraints,
            idealStartingState,
            finalGoal,
            false);

    activeFollowSampled = sampled;
    activeFollowCum = cum;
    activeFollowTotalLength = totalLength;
    activeFollowWaypointSlots = waypointSlots;
    activeExplicitRotationSamples = rotationSamples;
    activeFollowWindows = followWindows;

    Command followCmd = AutoBuilder.followPath(finalPath);

    if (!activeFollowEnabled) {
      clearFollowRotationOverride();
      return followCmd;
    }

    return Commands.sequence(
            Commands.runOnce(PathBuilder::installFollowRotationOverride), followCmd)
        .finallyDo(PathBuilder::clearFollowRotationOverride);
  }

  private static Target getFirstOverrideTarget(List<Target> activeTargets) {
    for (Target target : activeTargets) {
      if (target.overrideRotations != null && target.overrideRotations.length > 0) {
        return target;
      }
    }
    return null;
  }

  private static List<RotationTarget> buildOverrideRotationTargets(
      List<Target> activeTargets,
      double waypointSlots,
      Rotation2d startingRotation,
      Rotation2d endingRotation) {

    Target source = getFirstOverrideTarget(activeTargets);
    if (source == null) {
      return Collections.emptyList();
    }

    List<RotationTarget> output = new ArrayList<>();
    double endPos = Math.max(0.0, waypointSlots);

    if (startingRotation != null) {
      output.add(new RotationTarget(0.0, startingRotation));
    }

    if (source.overrideRotations != null) {
      for (RotationTarget target : source.overrideRotations) {
        if (target == null) {
          continue;
        }

        double pos = clamp(target.position(), 0.0, endPos);

        if (startingRotation != null && pos <= ROTATION_POSITION_EPS) {
          continue;
        }
        if (endingRotation != null && Math.abs(pos - endPos) <= ROTATION_POSITION_EPS) {
          continue;
        }

        output.add(new RotationTarget(pos, target.rotation()));
      }
    }

    if (endingRotation != null) {
      output.add(new RotationTarget(endPos, endingRotation));
    }

    output.sort(Comparator.comparingDouble(RotationTarget::position));

    List<RotationTarget> deduped = new ArrayList<>();
    for (RotationTarget target : output) {
      if (deduped.isEmpty()) {
        deduped.add(target);
        continue;
      }

      RotationTarget last = deduped.get(deduped.size() - 1);
      if (Math.abs(last.position() - target.position()) <= ROTATION_POSITION_EPS) {
        continue;
      }

      deduped.add(target);
    }

    return deduped;
  }

  private static List<RotationSample> rotationSamplesFromRotationTargets(
      List<RotationTarget> rotationTargets) {
    if (rotationTargets == null || rotationTargets.isEmpty()) {
      return Collections.emptyList();
    }

    List<RotationSample> samples = new ArrayList<>(rotationTargets.size());
    for (RotationTarget target : rotationTargets) {
      samples.add(new RotationSample(target.position(), target.rotation()));
    }
    return samples;
  }

  private static List<Waypoint> waypointsFromPosesWithCurve(
      List<Pose2d> poses, List<Target> targets) {
    List<Waypoint> waypoints = new ArrayList<>(poses.size());

    for (int i = 0; i < poses.size(); i++) {
      Pose2d pose = poses.get(i);
      Translation2d anchor = pose.getTranslation();
      Target target = targets.get(i);

      Rotation2d travelHeading = target.heading != null ? target.heading : pose.getRotation();
      double curveScale = clamp01(target.curveScale);

      Translation2d prevControl = anchor;
      Translation2d nextControl = anchor;

      if (poses.size() > 1) {
        Translation2d dir = new Translation2d(travelHeading.getCos(), travelHeading.getSin());

        double prevDist = 0.0;
        double nextDist = 0.0;

        if (i > 0) {
          prevDist = anchor.getDistance(poses.get(i - 1).getTranslation()) * 0.33 * curveScale;
        }
        if (i < poses.size() - 1) {
          nextDist = anchor.getDistance(poses.get(i + 1).getTranslation()) * 0.33 * curveScale;
        }

        if (target.controlDistanceBeforeMeters != null) {
          prevDist = target.controlDistanceBeforeMeters;
        }
        if (target.controlDistanceAfterMeters != null) {
          nextDist = target.controlDistanceAfterMeters;
        }

        prevControl = anchor.minus(dir.times(prevDist));
        nextControl = anchor.plus(dir.times(nextDist));
      }

      waypoints.add(new Waypoint(prevControl, anchor, nextControl));
    }

    return waypoints;
  }

  private static void installFollowRotationOverride() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    if (!activeFollowEnabled) {
      return;
    }

    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          if (activeFollowSampled == null || activeFollowSampled.isEmpty()) {
            return 0.0;
          }

          Pose2d pose = getPose.get();
          Rotation2d desired = getRuntimeDesiredRotation(pose);

          angleController.enableContinuousInput(-Math.PI, Math.PI);

          if (logged) {
            Logger.recordOutput("PathBuilder/Track Target Angle", desired.getRadians());
            Logger.recordOutput("PathBuilder/Track Current Angle", pose.getRotation().getRadians());
          }

          double omega =
              angleController.calculate(pose.getRotation().getRadians(), desired.getRadians())
                  + angleController.getSetpoint().velocity * angleFF;

          if (Math.abs(normalizeAngle(pose.getRotation().getRadians() - desired.getRadians()))
                  < angleTol.getRadians()
              && angleController.getSetpoint().velocity == 0.0) {
            omega = 0.0;
          }

          return omega;
        });
  }

  private static void clearFollowRotationOverride() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    activeFollowEnabled = false;
    activeFollowWindows = Collections.emptyList();
    activeExplicitRotationSamples = Collections.emptyList();
    activeFollowSampled = Collections.emptyList();
    activeFollowCum = new double[0];
    activeFollowTotalLength = 0.0;
    activeFollowWaypointSlots = 1.0;
  }

  private static Rotation2d getRuntimeDesiredRotation(Pose2d pose) {
    double arc = getClosestArc(pose.getTranslation(), activeFollowSampled, activeFollowCum);

    FollowWindow window = getBestFollowWindow(arc);
    if (window != null) {
      Rotation2d tangent =
          getLookaheadTangentAtArc(arc, window.lookaheadMeters, window.spread, window.sampleMeters);
      Rotation2d explicit = getExplicitRotationAtArc(arc);
      double blend = smoothstep(window.startArc, window.endArc, arc) * window.tangentWeight;
      return interpolateRotation(explicit, tangent, blend);
    }

    return getExplicitRotationAtArc(arc);
  }

  private static FollowWindow getBestFollowWindow(double arc) {
    FollowWindow best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    for (FollowWindow window : activeFollowWindows) {
      if (arc + 1e-9 < window.startArc || arc - 1e-9 > window.endArc) {
        continue;
      }

      double center = 0.5 * (window.startArc + window.endArc);
      double score = Math.abs(arc - center);
      if (score < bestScore) {
        bestScore = score;
        best = window;
      }
    }

    return best;
  }

  private static FollowProfile computeFollowProfile(
      List<Pose2d> sampled, double[] cum, double arc, double totalLength, Target target) {
    double curvature = estimateLocalCurvature(sampled, cum, arc, totalLength);

    double speed = 0.0;
    if (getChassisSpeeds != null) {
      ChassisSpeeds chassisSpeeds = getChassisSpeeds.get();
      speed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    double baseLead =
        Math.max(
            0.15,
            Math.abs(target.rotationLeadMeters) > 1e-9
                ? target.rotationLeadMeters
                : FOLLOW_ROTATION_SAMPLE_METERS * 0.85);

    double lead = baseLead + (speed * 0.15) + (curvature * 0.45);
    double spread = Math.max(1.0, target.rotationSpread) + (curvature * 1.6) + (speed * 0.08);

    double sampleMeters =
        FOLLOW_ROTATION_SAMPLE_METERS / (1.0 + (curvature * 2.2) + (speed * 0.05));

    double tangentWeight = clamp01(Math.max(0.0, target.tangentWeight));

    return new FollowProfile(
        clamp(lead, 0.15, Math.max(0.15, totalLength * 0.25)),
        clamp(spread, 1.0, 4.0),
        clamp(sampleMeters, 0.08, 0.80),
        tangentWeight);
  }

  private static Rotation2d getLookaheadTangentAtArc(
      double arc, double lookaheadMeters, double spread, double sampleMeters) {

    double speedLead = 0.0;
    if (getChassisSpeeds != null) {
      ChassisSpeeds speeds = getChassisSpeeds.get();
      speedLead = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 0.15;
    }

    double curvatureLead =
        estimateLocalCurvature(activeFollowSampled, activeFollowCum, arc, activeFollowTotalLength)
            * 0.35;

    double lead =
        clamp(
            Math.max(lookaheadMeters, 0.15) + speedLead + curvatureLead,
            0.15,
            Math.max(0.15, activeFollowTotalLength * 0.25));

    double sampleWindow = clamp(sampleMeters, 0.08, Math.max(0.08, activeFollowTotalLength * 0.10));

    double behind = Math.max(0.0, arc - lead * (0.25 / Math.max(1.0, spread)));
    double ahead = Math.min(activeFollowTotalLength, arc + lead);

    Translation2d p0 = sampleAtArc(activeFollowSampled, activeFollowCum, behind);
    Translation2d p1 = sampleAtArc(activeFollowSampled, activeFollowCum, ahead);

    Translation2d q0 =
        sampleAtArc(activeFollowSampled, activeFollowCum, Math.max(0.0, arc - sampleWindow));
    Translation2d q1 =
        sampleAtArc(
            activeFollowSampled,
            activeFollowCum,
            Math.min(activeFollowTotalLength, arc + sampleWindow));

    double dx = (p1.getX() - p0.getX()) * 0.7 + (q1.getX() - q0.getX()) * 0.3;
    double dy = (p1.getY() - p0.getY()) * 0.7 + (q1.getY() - q0.getY()) * 0.3;

    return safeHeading(
        dx, dy, activeFollowSampled.get(activeFollowSampled.size() - 1).getRotation());
  }

  private static double estimateLocalCurvature(
      List<Pose2d> sampled, double[] cum, double arc, double totalLength) {
    if (sampled == null || sampled.size() < 3) {
      return 0.0;
    }

    double delta = Math.max(0.08, FOLLOW_ROTATION_SAMPLE_METERS * 0.4);
    double a0 = Math.max(0.0, arc - delta);
    double a1 = arc;
    double a2 = Math.min(totalLength, arc + delta);

    Translation2d p0 = sampleAtArc(sampled, cum, a0);
    Translation2d p1 = sampleAtArc(sampled, cum, a1);
    Translation2d p2 = sampleAtArc(sampled, cum, a2);

    Rotation2d h1 = safeHeading(p1.getX() - p0.getX(), p1.getY() - p0.getY(), p1.getAngle());
    Rotation2d h2 = safeHeading(p2.getX() - p1.getX(), p2.getY() - p1.getY(), p2.getAngle());

    return Math.abs(normalizeAngle(h2.getRadians() - h1.getRadians()));
  }

  private static Rotation2d getExplicitRotationAtArc(double arc) {
    if (activeExplicitRotationSamples == null || activeExplicitRotationSamples.isEmpty()) {
      if (activeFollowSampled != null && !activeFollowSampled.isEmpty()) {
        return activeFollowSampled.get(activeFollowSampled.size() - 1).getRotation();
      }
      return Rotation2d.fromDegrees(0.0);
    }

    double pos =
        (activeFollowTotalLength <= 1e-9)
            ? 0.0
            : (arc / activeFollowTotalLength) * activeFollowWaypointSlots;
    pos = Math.max(0.0, Math.min(activeFollowWaypointSlots, pos));

    RotationSample prev = activeExplicitRotationSamples.get(0);
    if (pos <= prev.position) {
      return prev.rotation;
    }

    for (int i = 1; i < activeExplicitRotationSamples.size(); i++) {
      RotationSample next = activeExplicitRotationSamples.get(i);
      if (pos <= next.position + 1e-9) {
        double span = next.position - prev.position;
        if (span < 1e-9) {
          return next.rotation;
        }

        double t = (pos - prev.position) / span;
        return interpolateRotation(prev.rotation, next.rotation, t);
      }
      prev = next;
    }

    return activeExplicitRotationSamples.get(activeExplicitRotationSamples.size() - 1).rotation;
  }

  private static Rotation2d emitRotationTarget(
      List<RotationTarget> rotationTargets,
      double arc,
      Rotation2d desired,
      Rotation2d lastRotation,
      double totalLength,
      double waypointSlots,
      boolean force) {

    if (!force
        && Math.abs(normalizeAngle(desired.getRadians() - lastRotation.getRadians()))
            <= ROTATION_TOL_RAD) {
      return lastRotation;
    }

    double pos = (arc / totalLength) * waypointSlots;
    pos = Math.max(0.0, Math.min(waypointSlots, pos));

    if (!rotationTargets.isEmpty()) {
      RotationTarget last = rotationTargets.get(rotationTargets.size() - 1);
      if (pos <= last.position() + 1e-6) {
        pos = Math.min(waypointSlots, last.position() + 1e-6);
      }
    }

    rotationTargets.add(new RotationTarget(pos, desired));
    return desired;
  }

  private static Rotation2d getBiasedTangentAtArc(
      List<Pose2d> sampled, double[] cum, double arc, double totalLength, double sampleMeters) {

    double lookahead = clamp(sampleMeters, 0.08, 0.80);

    double aheadArc = Math.min(totalLength, arc + lookahead);
    double behindArc = Math.max(0.0, arc - lookahead * 0.25);

    Translation2d p0 = sampleAtArc(sampled, cum, behindArc);
    Translation2d p1 = sampleAtArc(sampled, cum, aheadArc);

    double dx = p1.getX() - p0.getX();
    double dy = p1.getY() - p0.getY();

    return new Rotation2d(Math.atan2(dy, dx));
  }

  private static Translation2d sampleAtArc(List<Pose2d> sampled, double[] cum, double arc) {
    for (int i = 0; i < cum.length - 1; i++) {
      if (arc >= cum[i] && arc <= cum[i + 1]) {
        double t = (arc - cum[i]) / (cum[i + 1] - cum[i] + 1e-9);

        Translation2d a = sampled.get(i).getTranslation();
        Translation2d b = sampled.get(i + 1).getTranslation();

        return new Translation2d(
            a.getX() + (b.getX() - a.getX()) * t, a.getY() + (b.getY() - a.getY()) * t);
      }
    }

    return sampled.get(sampled.size() - 1).getTranslation();
  }

  private static double getClosestArc(Translation2d point, List<Pose2d> sampled, double[] cum) {
    if (sampled == null || sampled.isEmpty()) {
      return 0.0;
    }
    if (sampled.size() == 1) {
      return 0.0;
    }

    double bestDist = Double.POSITIVE_INFINITY;
    double bestArc = 0.0;

    for (int i = 0; i < sampled.size() - 1; i++) {
      Translation2d a = sampled.get(i).getTranslation();
      Translation2d b = sampled.get(i + 1).getTranslation();

      double ax = a.getX();
      double ay = a.getY();
      double bx = b.getX();
      double by = b.getY();
      double dx = bx - ax;
      double dy = by - ay;
      double segLenSq = dx * dx + dy * dy;

      double u = 0.0;
      if (segLenSq > 1e-12) {
        double tx = point.getX() - ax;
        double ty = point.getY() - ay;
        u = (tx * dx + ty * dy) / segLenSq;
        if (u < 0.0) {
          u = 0.0;
        } else if (u > 1.0) {
          u = 1.0;
        }
      }

      double projX = ax + u * dx;
      double projY = ay + u * dy;
      double dist = Math.hypot(point.getX() - projX, point.getY() - projY);

      if (dist < bestDist) {
        bestDist = dist;
        double segLen = Math.hypot(dx, dy);
        bestArc = cum[i] + segLen * u;
      }
    }

    return bestArc;
  }

  private static List<ConstraintsZone> mergeConstraintsZones(List<ConstraintsZone> zones) {
    if (zones == null || zones.size() <= 1) {
      return zones;
    }
    zones.sort(Comparator.comparingDouble(ConstraintsZone::minPosition));
    List<ConstraintsZone> out = new ArrayList<>();
    ConstraintsZone cur = zones.get(0);
    for (int i = 1; i < zones.size(); i++) {
      ConstraintsZone next = zones.get(i);
      boolean sameConstr = Objects.equals(cur.constraints(), next.constraints());
      if (sameConstr && next.minPosition() <= cur.maxPosition() + 1e-9) {
        cur =
            new ConstraintsZone(
                cur.minPosition(),
                Math.max(cur.maxPosition(), next.maxPosition()),
                cur.constraints());
      } else {
        out.add(cur);
        cur = next;
      }
    }
    out.add(cur);
    return out;
  }

  private static double normalizeAngle(double a) {
    return Math.atan2(Math.sin(a), Math.cos(a));
  }

  private static double clamp01(double value) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    return Math.max(0.0, Math.min(1.0, value));
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static double smoothstep(double edge0, double edge1, double x) {
    if (edge1 <= edge0) {
      return clamp01(x >= edge0 ? 1.0 : 0.0);
    }
    double t = clamp01((x - edge0) / (edge1 - edge0));
    return t * t * (3.0 - 2.0 * t);
  }

  private static Rotation2d safeHeading(double dx, double dy, Rotation2d fallback) {
    final double EPS = 1e-6;
    if (Math.abs(dx) < EPS && Math.abs(dy) < EPS) {
      return fallback != null ? fallback : Rotation2d.fromDegrees(0.0);
    }
    return new Rotation2d(dx, dy);
  }

  private static Rotation2d interpolateRotation(Rotation2d from, Rotation2d to, double t) {
    t = clamp01(t);
    double delta = normalizeAngle(to.getRadians() - from.getRadians());
    return new Rotation2d(from.getRadians() + delta * t);
  }

  // My own auto triggers :) very simple Commands but maintains a uniform structure through the
  // syntax

  public static Command triggerWhenClose(
      Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(
            () ->
                getPose.get().getTranslation().getDistance(AllianceFlip.apply(location))
                    <= distance)
        .andThen(runnable);
  }

  public static Command triggerWhenFar(Translation2d location, double distance, Command runnable) {
    return Commands.waitUntil(
            () ->
                getPose.get().getTranslation().getDistance(AllianceFlip.apply(location)) > distance)
        .andThen(runnable);
  }

  public static Command triggerWhenTrue(BooleanSupplier condition, Command runnable) {
    return Commands.waitUntil(condition).andThen(runnable);
  }

  public static Command triggerWithDelay(double seconds, Command runnable) {
    return Commands.waitSeconds(seconds).andThen(runnable);
  }

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
   * Merges into a path using Pathfinding
   *
   * @param knownPath PathPlannerPath
   * @return Command
   */
  public static Command mergeToPath(PathPlannerPath knownPath) {
    return AutoBuilder.pathfindThenFollowPath(knownPath, constraints)
        .beforeStarting(() -> angleController.reset(getPose.get().getRotation().getRadians()))
        .finallyDo(stopDrive);
  }
}
