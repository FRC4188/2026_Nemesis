package frc.robot.CSPLib.ppp;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
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
public final class PBExperimental {
  private static Drive drive;

  // prob too fast
  private static PathConstraints constraints =
      new PathConstraints(
          Constants.DriveConstants.DRIVE_MAXVEL * 0.8,
          Constants.DriveConstants.DRIVE_MAXACC * 0.8,
          Constants.DriveConstants.ANGLE_MAXVEL * 0.8,
          Constants.DriveConstants.ANGLE_MAXACC * 0.8);

  private static double ROTATION_TOL_RAD = Math.toRadians(30);
  private static double FOLLOW_ROTATION_SAMPLE_METERS = 1.5;
  private static double PATH_CREATION_TOL = Units.inchesToMeters(1);

  private static boolean logged;
  private static double angleFF;
  private static Rotation2d angleTol;
  private static Supplier<Pose2d> getPose;
  private static Consumer<Pose2d> setPose;
  private static Supplier<ChassisSpeeds> getChassisSpeeds;
  private static Consumer<ChassisSpeeds> runVelocity;
  private static ProfiledPIDController driveController;
  private static ProfiledPIDController angleController;
  private static RobotConfig robotConfig;
  private static BooleanSupplier shouldFlip;
  private static Runnable stopDrive;

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
    setPose = setPose_;
    getChassisSpeeds = getChassisSpeeds_;
    runVelocity = runVelocity_;
    driveController = driveController_;
    angleController = angleController_;
    robotConfig = robotConfig_;
    shouldFlip = shouldFlip_;
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
          (activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
          });

      PathPlannerLogging.setLogTargetPoseCallback(
          (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
          });
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
              && angleController.getSetpoint().velocity == 0.0) omega = 0.0;

          return omega;
        });
  }

  /*
   * Stops any current rotation tracking
   */
  public static void stopTarget() {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
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
    public final BooleanSupplier condition;
    public final double toleranceMeters;
    public final Command command;

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
        double toleranceMeters,
        Command command,
        BooleanSupplier condition) {
      if (pose == null) throw new IllegalArgumentException("pose cannot be null");
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
      if (!Double.isFinite(tangentWeight)) {
        throw new IllegalArgumentException("tangentWeight must be finite");
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
          toleranceMeters,
          command,
          condition);
    }

    public Target withHeading(Rotation2d heading) {
      if (heading == null) throw new IllegalArgumentException("heading cannot be null");
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
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
          toleranceMeters,
          command,
          condition);
    }

    public Target onlyIf(BooleanSupplier condition) {
      if (condition == null) throw new IllegalArgumentException("condition cannot be null");
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
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
          toleranceMeters,
          command,
          condition);
    }

    public Target withCommand(Command command) {
      return new Target(
          pose,
          speedMultiplier,
          rotationSpeedMultiplier,
          rotationLeadMeters,
          rotationSpread,
          rotationMode,
          heading,
          tangentWeight,
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
    for (int i = 0; i < activeTargets.size(); i++) poses[i] = activeTargets.get(i).pose;

    List<Pose2d> travelHeadingPoses = new ArrayList<>(poses.length);
    for (int i = 0; i < poses.length; i++) {
      Translation2d pos = poses[i].getTranslation();
      Rotation2d heading;
      if (poses.length == 1) {
        heading = poses[i].getRotation();
      } else if (i == 0) {
        Translation2d next = poses[i + 1].getTranslation();
        heading =
            safeHeading(next.getX() - pos.getX(), next.getY() - pos.getY(), poses[i].getRotation());
      } else if (i == poses.length - 1) {
        Translation2d prev = poses[i - 1].getTranslation();
        heading =
            safeHeading(pos.getX() - prev.getX(), pos.getY() - prev.getY(), poses[i].getRotation());
      } else {
        Translation2d prev = poses[i - 1].getTranslation();
        Translation2d next = poses[i + 1].getTranslation();
        heading =
            safeHeading(
                next.getX() - prev.getX(), next.getY() - prev.getY(), poses[i].getRotation());
      }

      Target target = activeTargets.get(i);
      Rotation2d preferredHeading =
          target.heading != null ? target.heading : poses[i].getRotation();
      double tangentWeight = Math.max(0.0, Math.min(1.0, target.tangentWeight));
      double blendedHeadingRadians =
          preferredHeading.getRadians()
              + normalizeAngle(heading.getRadians() - preferredHeading.getRadians())
                  * tangentWeight;

      travelHeadingPoses.add(new Pose2d(pos, new Rotation2d(blendedHeadingRadians)));
    }

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(travelHeadingPoses);
    final int waypointSlots = Math.max(1, waypoints.size() - 1);

    List<RotationTarget> emptyRotationTargets = Collections.<RotationTarget>emptyList();
    List<PointTowardsZone> emptyPointTowards = Collections.<PointTowardsZone>emptyList();
    List<ConstraintsZone> emptyConstraintsZones = Collections.<ConstraintsZone>emptyList();
    List<EventMarker> emptyEventMarkers = Collections.<EventMarker>emptyList();

    PathConstraints globalConstraints = getConstraints();

    GoalEndState tmpGoal = new GoalEndState(0.0, poses[poses.length - 1].getRotation());
    PathPlannerPath tempPath =
        new PathPlannerPath(
            waypoints,
            emptyRotationTargets,
            emptyPointTowards,
            emptyConstraintsZones,
            emptyEventMarkers,
            globalConstraints,
            null,
            tmpGoal,
            false);

    List<Pose2d> sampled = tempPath.getPathPoses();
    if (sampled == null || sampled.isEmpty()) {
      List<RotationTarget> fallbackRT = new ArrayList<>();
      List<EventMarker> fallbackMarkers = new ArrayList<>();
      int n = poses.length;

      fallbackRT.add(new RotationTarget(0.0, poses[0].getRotation()));
      Rotation2d lastFallbackRotation = poses[0].getRotation();

      for (int i = 1; i < n; i++) {
        double pct = (n == 1) ? 0.0 : ((double) i) / (n - 1);
        Target target = activeTargets.get(i);

        if (target.command != null) {
          fallbackMarkers.add(new EventMarker("target_" + i, pct * waypointSlots, target.command));
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

      GoalEndState finalGoal = new GoalEndState(0.0, lastFallbackRotation);
      PathPlannerPath fallback =
          new PathPlannerPath(
              waypoints,
              fallbackRT,
              emptyPointTowards,
              emptyConstraintsZones,
              fallbackMarkers.isEmpty() ? emptyEventMarkers : fallbackMarkers,
              globalConstraints,
              null,
              finalGoal,
              false);
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
    if (totalLength <= 1e-9) totalLength = 1e-9;

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
        if (targetArcs[t] < lastArcAccepted) targetArcs[t] = lastArcAccepted;
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
            if (u < 0.0) u = 0.0;
            else if (u > 1.0) u = 1.0;
          }

          double projX = ax + u * dx;
          double projY = ay + u * dy;
          double dist = Math.hypot(targetTrans.getX() - projX, targetTrans.getY() - projY);

          double segLen = Math.hypot(dx, dy);
          double arcAlong = cum[i] + (segLen * u);

          if (arcAlong + 1e-9 < lastArcAccepted) continue;

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
            if (u < 0.0) u = 0.0;
            else if (u > 1.0) u = 1.0;
          }
          double segLen = Math.hypot(dx, dy);
          bestArc = cum[segIdx] + (segLen * u);
          bestSeg = segIdx;
        }
      }

      if (bestArc < lastArcAccepted) bestArc = lastArcAccepted;
      if (bestArc > totalLength) bestArc = totalLength;
      targetArcs[t] = bestArc;
      lastArcAccepted = bestArc;
      searchSegStart = Math.min(bestSeg, sampled.size() - 2);
    }

    List<RotationTarget> rotationTargets = new ArrayList<>();
    rotationTargets.add(new RotationTarget(0.0, poses[0].getRotation()));
    Rotation2d lastRotation = poses[0].getRotation();
    final double PRE_OFFSET_ARC = Math.max(1e-3, totalLength * 1e-6);

    List<EventMarker> eventMarkers = new ArrayList<>();

    for (int t = 1; t < nTargets; t++) {
      Target target = activeTargets.get(t);
      double arc = targetArcs[t];

      if (target.command != null) {
        double markerPos = (arc / totalLength) * waypointSlots;
        eventMarkers.add(new EventMarker("target_" + t, markerPos, target.command));
      }

      switch (target.rotationMode) {
        case FOLLOW:
          {
            Rotation2d offset = poses[t].getRotation();

            double leadMeters = target.rotationLeadMeters;
            double rotationStartArc = arc - leadMeters;
            rotationStartArc = Math.max(0.0, Math.min(rotationStartArc, totalLength));

            double baseEndArc = (leadMeters >= 0.0) ? arc : (arc - leadMeters);
            baseEndArc = Math.max(0.0, Math.min(baseEndArc, totalLength));

            double baseWindow = Math.abs(baseEndArc - rotationStartArc);
            if (baseWindow < 1e-12) baseWindow = PRE_OFFSET_ARC;

            double spread = target.rotationSpread;
            if (!Double.isFinite(spread) || spread <= 0.0) spread = 1.0;

            double spreadWindow = baseWindow * spread;
            double rotationEndArc = rotationStartArc + spreadWindow;
            rotationEndArc = Math.max(0.0, Math.min(rotationEndArc, totalLength));

            lastRotation =
                addDenseFollowRotationTargets(
                    rotationTargets,
                    sampled,
                    cum,
                    totalLength,
                    rotationStartArc,
                    rotationEndArc,
                    offset,
                    lastRotation,
                    waypointSlots);
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
              break;
            }

            double leadMeters = target.rotationLeadMeters;
            double rotationStartArc = arc - leadMeters;
            rotationStartArc = Math.max(0.0, Math.min(rotationStartArc, totalLength));

            double baseEndArc = (leadMeters >= 0.0) ? arc : (arc - leadMeters);
            baseEndArc = Math.max(0.0, Math.min(baseEndArc, totalLength));

            double baseWindow = Math.abs(baseEndArc - rotationStartArc);
            if (baseWindow < 1e-12) baseWindow = PRE_OFFSET_ARC;

            double spread = target.rotationSpread;
            if (!Double.isFinite(spread) || spread <= 0.0) spread = 1.0;

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
            rotationTargets.add(new RotationTarget(finalWaypointPos, desired));
            lastRotation = desired;
            break;
          }
      }
    }

    List<ConstraintsZone> constraintsZones = new ArrayList<>();
    final double ZONE_EPSILON = 1e-6;
    for (int i = 0; i < nTargets; i++) {
      Target target = activeTargets.get(i);

      boolean linearChanged = Math.abs(target.speedMultiplier - 1.0) > 1e-9;
      boolean angularChanged = Math.abs(target.rotationSpeedMultiplier - 1.0) > 1e-9;
      if (!linearChanged && !angularChanged) continue;

      if (i > 0) {
        Target prev = activeTargets.get(i - 1);
        boolean prevChanged =
            Math.abs(prev.speedMultiplier - 1.0) > 1e-9
                || Math.abs(prev.rotationSpeedMultiplier - 1.0) > 1e-9;
        if (prevChanged) {
          continue;
        }
      }

      int j = i + 1;
      while (j < nTargets) {
        Target next = activeTargets.get(j);
        boolean nextChanged =
            Math.abs(next.speedMultiplier - 1.0) > 1e-9
                || Math.abs(next.rotationSpeedMultiplier - 1.0) > 1e-9;
        if (!nextChanged) break;
        j++;
      }

      double startArc = targetArcs[i];
      double endArc = (j < nTargets) ? targetArcs[j] : totalLength;

      if (endArc - startArc < ZONE_EPSILON) {
        endArc = Math.min(totalLength, startArc + 1e-3);
        if (endArc - startArc < ZONE_EPSILON) {
          startArc = Math.max(0.0, startArc - 1e-3);
        }
      }

      double startFrac = startArc / totalLength;
      double endFrac = endArc / totalLength;

      double startWaypointPos = Math.max(0.0, Math.min(waypointSlots, startFrac * waypointSlots));
      double endWaypointPos = Math.max(0.0, Math.min(waypointSlots, endFrac * waypointSlots));

      double linearMult = target.speedMultiplier;
      double angularMult = target.rotationSpeedMultiplier;

      PathConstraints scaled =
          new PathConstraints(
              Math.max(0.0, globalConstraints.maxVelocityMPS() * linearMult),
              Math.max(0.0, globalConstraints.maxAccelerationMPSSq() * linearMult),
              Math.max(0.0, globalConstraints.maxAngularVelocityRadPerSec() * angularMult),
              Math.max(0.0, globalConstraints.maxAngularAccelerationRadPerSecSq() * angularMult));

      constraintsZones.add(new ConstraintsZone(startWaypointPos, endWaypointPos, scaled));
    }

    constraintsZones = mergeConstraintsZones(constraintsZones);

    GoalEndState finalGoal = new GoalEndState(0.0, lastRotation);
    PathPlannerPath finalPath =
        new PathPlannerPath(
            waypoints,
            rotationTargets,
            emptyPointTowards,
            constraintsZones.isEmpty() ? emptyConstraintsZones : constraintsZones,
            eventMarkers.isEmpty() ? emptyEventMarkers : eventMarkers,
            globalConstraints,
            null,
            finalGoal,
            false);

    return AutoBuilder.followPath(finalPath);
  }

  private static Rotation2d addDenseFollowRotationTargets(
      List<RotationTarget> rotationTargets,
      List<Pose2d> sampled,
      double[] cum,
      double totalLength,
      double startArc,
      double endArc,
      Rotation2d offset,
      Rotation2d lastRotation,
      double waypointSlots) {

    if (endArc < startArc) {
      double tmp = startArc;
      startArc = endArc;
      endArc = tmp;
    }

    startArc = Math.max(0.0, Math.min(startArc, totalLength));
    endArc = Math.max(0.0, Math.min(endArc, totalLength));

    double window = endArc - startArc;
    if (window < 1e-9) {
      Rotation2d desired = getBiasedTangentAtArc(sampled, cum, startArc, totalLength).plus(offset);
      return emitRotationTarget(
          rotationTargets, startArc, desired, lastRotation, totalLength, waypointSlots, true);
    }

    double stepMeters = Math.min(FOLLOW_ROTATION_SAMPLE_METERS, Math.max(0.10, window / 4.0));

    double arc = startArc;
    while (arc < endArc - 1e-9) {
      Rotation2d tangent = getBiasedTangentAtArc(sampled, cum, arc, totalLength);
      Rotation2d desired = tangent.plus(offset);

      lastRotation =
          emitRotationTarget(
              rotationTargets, arc, desired, lastRotation, totalLength, waypointSlots, false);

      arc += stepMeters;
    }

    Rotation2d endDesired = getBiasedTangentAtArc(sampled, cum, endArc, totalLength).plus(offset);
    return emitRotationTarget(
        rotationTargets, endArc, endDesired, lastRotation, totalLength, waypointSlots, true);
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
      List<Pose2d> sampled, double[] cum, double arc, double totalLength) {

    double lookahead = 0.4; // meters (0.2–0.8 is typical)

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

  private static List<ConstraintsZone> mergeConstraintsZones(List<ConstraintsZone> zones) {
    if (zones == null || zones.size() <= 1) return zones;
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

  private static Rotation2d safeHeading(double dx, double dy, Rotation2d fallback) {
    final double EPS = 1e-6;
    if (Math.abs(dx) < EPS && Math.abs(dy) < EPS) {
      return fallback != null ? fallback : Rotation2d.fromDegrees(0.0);
    }
    return new Rotation2d(dx, dy);
  }

  // My own auto triggers :) very simple Commands but maintains a uniform structure through the
  // syntax

  public static Command triggerWhenClose(
      Translation2d location, double distance, Command runnable) {
    System.out.println(getPose.get());

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
