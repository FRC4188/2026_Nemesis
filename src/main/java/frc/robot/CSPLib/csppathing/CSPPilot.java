package frc.robot.CSPLib.csppathing;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.UnaryOperator;

/**
 * A pose-parameterized motion controller, switching time to pose parameterization. Best to be used
 * with CSPPathing.
 */
public class CSPPilot {
  private static final double GRAVITY = 9.81;

  private final CSPProfile profile;
  private final double dt = 0.020;

  public CSPPilot(CSPProfile profile) {
    this.profile = Objects.requireNonNull(profile, "profile cannot be null");
  }

  public CSPProfile getProfile() {
    return profile;
  }

  public record PathSeed(CSPConstraints constraints, double startingSpeed, double endingSpeed) {}

  public PathSeed seedFrom(PathPlannerPath path) {
    return seedFrom(path, null);
  }

  public PathSeed seedFrom(PathPlannerPath path, RobotConfig robotConfig) {
    Objects.requireNonNull(path, "path cannot be null");

    double startingSpeed = extractStartingSpeed(path);
    double endingSpeed = extractEndingSpeed(path);

    CSPConstraints constraints = profile.getConstraints();

    PathConstraints global = path.getGlobalConstraints();
    if (global != null && !global.unlimited()) {
      constraints = mergeConstraints(constraints, global);
    }

    if (robotConfig != null) {
      constraints = mergeConstraints(constraints, robotConfig);
    }

    return new PathSeed(constraints, 1000, 1000);
  }

  public CSPResult calculate(Pose2d current, ChassisSpeeds robotRelativeSpeeds, CSPTarget target) {
    return calculate(
        current,
        robotRelativeSpeeds,
        target,
        profile.getConstraints(),
        profile.getConstraints().velocity);
  }

  private CSPResult calculate(
      Pose2d current,
      ChassisSpeeds robotRelativeSpeeds,
      CSPTarget target,
      CSPConstraints activeConstraints,
      double speedCap) {

    Translation2d offset =
        toTargetCoordinateFrame(
            target.getReference().getTranslation().minus(current.getTranslation()), target);

    if (offset.getNorm() < 1e-9) {
      return new CSPResult(
          MetersPerSecond.of(0), MetersPerSecond.of(0), target.getReference().getRotation());
    }

    Translation2d fieldRelativeSpeeds =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(current.getRotation());

    Translation2d initial = toTargetCoordinateFrame(fieldRelativeSpeeds, target);

    double disp = offset.getNorm();
    double velocityCap = activeConstraints.velocity;
    if (Double.isFinite(speedCap)) {
      velocityCap = Math.min(velocityCap, Math.max(0.0, speedCap));
    }

    if (target.getEntryAngle().isEmpty() || disp < profile.getBeelineRadius().in(Meters)) {
      Translation2d towardsTarget = offset.div(disp);

      double goalLimit = Math.min(activeConstraints.calculateMaxVelocity(disp), velocityCap);
      if (target.getVelocity() > 1e-9) {
        goalLimit = Math.min(goalLimit, target.getVelocity());
      }

      Translation2d goal = towardsTarget.times(goalLimit);
      Translation2d out = correct(initial, goal, activeConstraints, velocityCap);
      Translation2d velo = toGlobalCoordinateFrame(out, target);
      Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);

      return new CSPResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
    }

    double goalLimit = Math.min(activeConstraints.calculateMaxVelocity(disp), velocityCap);
    if (target.getVelocity() > 1e-9) {
      goalLimit = Math.min(goalLimit, target.getVelocity());
    }

    Translation2d goal = calculateSwirlyVelocity(offset, goalLimit);
    Translation2d out = correct(initial, goal, activeConstraints, velocityCap);
    Translation2d velo = toGlobalCoordinateFrame(out, target);
    Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);

    return new CSPResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
  }

  public PathFollower followPath(PathPlannerPath path) {
    PathSeed seed = seedFrom(path, null);
    return followPath(path, seed.constraints(), seed.startingSpeed(), seed.endingSpeed());
  }

  public PathFollower followPath(PathPlannerPath path, double startingSpeed, double endingSpeed) {
    PathSeed seed = seedFrom(path, null);
    return followPath(path, seed.constraints(), startingSpeed, endingSpeed);
  }

  public PathFollower followPath(PathPlannerPath path, RobotConfig robotConfig) {
    PathSeed seed = seedFrom(path, robotConfig);
    return followPath(path, seed.constraints(), seed.startingSpeed(), seed.endingSpeed());
  }

  public PathFollower followPath(
      PathPlannerPath path, RobotConfig robotConfig, double startingSpeed, double endingSpeed) {
    PathSeed seed = seedFrom(path, robotConfig);
    return followPath(path, seed.constraints(), startingSpeed, endingSpeed);
  }

  public PathFollower followPath(
      PathPlannerPath path, CSPConstraints constraints, double startingSpeed, double endingSpeed) {
    return new PathFollower(path, constraints, startingSpeed, endingSpeed);
  }

  public PathFollower followPath(PathPlannerPath path, PathSeed seed) {
    return new PathFollower(path, seed.constraints(), seed.startingSpeed(), seed.endingSpeed());
  }

  private CSPConstraints mergeConstraints(CSPConstraints base, PathConstraints pathConstraints) {
    if (pathConstraints == null || pathConstraints.unlimited()) {
      return base;
    }

    return new CSPConstraints(
        Math.min(base.velocity, pathConstraints.maxVelocity().in(MetersPerSecond)),
        Math.min(base.acceleration, pathConstraints.maxAcceleration().in(MetersPerSecondPerSecond)),
        base.jerk);
  }

  private CSPConstraints mergeConstraints(CSPConstraints base, RobotConfig robotConfig) {
    if (robotConfig == null || robotConfig.moduleConfig == null) {
      return base;
    }

    double maxVelocity = robotConfig.moduleConfig.maxDriveVelocityMPS;

    double tractionAccel = Double.POSITIVE_INFINITY;
    if (Double.isFinite(robotConfig.massKG) && robotConfig.massKG > 1e-9) {
      tractionAccel = robotConfig.wheelFrictionForce / robotConfig.massKG;
    }
    if (!(tractionAccel > 1e-9) || Double.isNaN(tractionAccel)) {
      double wheelCof = robotConfig.moduleConfig.wheelCOF;
      tractionAccel = Double.isFinite(wheelCof) ? wheelCof * GRAVITY : Double.POSITIVE_INFINITY;
    }

    return new CSPConstraints(
        Math.min(base.velocity, maxVelocity),
        Math.min(base.acceleration, tractionAccel),
        base.jerk);
  }

  private double extractStartingSpeed(PathPlannerPath path) {
    if (path == null || path.getIdealStartingState() == null) {
      return 0.0;
    }
    return Math.max(0.0, path.getIdealStartingState().velocityMPS());
  }

  private double extractEndingSpeed(PathPlannerPath path) {
    if (path == null || path.getGoalEndState() == null) {
      return 0.0;
    }
    return Math.max(0.0, path.getGoalEndState().velocityMPS());
  }

  private Translation2d toTargetCoordinateFrame(Translation2d coords, CSPTarget target) {
    Rotation2d entryAngle = target.getEntryAngle().orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle.unaryMinus());
  }

  private Translation2d toGlobalCoordinateFrame(Translation2d coords, CSPTarget target) {
    Rotation2d entryAngle = target.getEntryAngle().orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle);
  }

  private Translation2d correct(
      Translation2d initial,
      Translation2d goal,
      CSPConstraints activeConstraints,
      double speedCap) {

    Rotation2d angleOffset = Rotation2d.kZero;
    if (goal.getNorm() > 1e-9) {
      angleOffset = new Rotation2d(goal.getX(), goal.getY());
    }

    Translation2d adjustedGoal = goal.rotateBy(angleOffset.unaryMinus());
    Translation2d adjustedInitial = initial.rotateBy(angleOffset.unaryMinus());

    double initialI = adjustedInitial.getX();
    double goalI = adjustedGoal.getX();

    double velocityCap = activeConstraints.velocity;
    if (Double.isFinite(speedCap)) {
      velocityCap = Math.min(velocityCap, Math.max(0.0, speedCap));
    }

    if (goalI > velocityCap) {
      goalI = velocityCap;
    }

    double adjustedI = Math.min(goalI, push(initialI, goalI, activeConstraints.acceleration));
    return new Translation2d(adjustedI, 0).rotateBy(angleOffset);
  }

  private double push(double start, double end, double accel) {
    double maxChange = accel * dt;
    if (Math.abs(start - end) < maxChange) {
      return end;
    }
    if (start > end) {
      return start - maxChange;
    }
    return start + maxChange;
  }

  private Translation2d calculateSwirlyVelocity(Translation2d offset, double speedLimit) {
    double disp = offset.getNorm();
    Rotation2d theta = new Rotation2d(offset.getX(), offset.getY());
    double rads = theta.getRadians();
    calculateSwirlyLength(rads, disp); // kept for parity with your original structure

    double vx = theta.getCos() - rads * theta.getSin();
    double vy = rads * theta.getCos() + theta.getSin();

    double norm = Math.hypot(vx, vy);
    if (norm < 1e-9) {
      return new Translation2d(0, 0);
    }

    return new Translation2d(vx, vy).div(norm).times(Math.max(0.0, speedLimit));
  }

  private double calculateSwirlyLength(double theta, double radius) {
    if (Math.abs(theta) < 1e-9) {
      return radius;
    }
    theta = Math.abs(theta);
    double hypot = Math.hypot(theta, 1);
    double u1 = radius * hypot;
    double u2 = radius * Math.log(theta + hypot) / theta;
    return 0.5 * (u1 + u2);
  }

  private Rotation2d getRotationTarget(Rotation2d current, CSPTarget target, double dist) {
    if (target.getRotationRadius().isEmpty()) {
      return target.getReference().getRotation();
    }
    double radius = target.getRotationRadius().get().in(Meters);
    if (radius > dist) {
      return target.getReference().getRotation();
    }
    return current;
  }

  public boolean atTarget(Pose2d current, CSPTarget target) {
    Pose2d goal = target.getReference();
    boolean okXY =
        Math.hypot(current.getX() - goal.getX(), current.getY() - goal.getY())
            <= profile.getErrorXY().in(Meters);
    boolean okTheta =
        Math.abs(current.getRotation().minus(goal.getRotation()).getRadians())
            <= profile.getErrorTheta().in(Radians);
    return okXY && okTheta;
  }

  public static PathPlannerPath createCustomPath(
      PathPlannerPath original, UnaryOperator<List<Pose2d>> manipulator) {
    return createCustomPath(original, 0, manipulator);
  }

  public static PathPlannerPath createCustomPath(
      PathPlannerPath original, int resampleCount, UnaryOperator<List<Pose2d>> manipulator) {
    CSPPath sampled = CSPPath.fromPathPlannerPath(original, resampleCount, manipulator);
    return sampled.toPathPlannerPath();
  }

  public record CSPResult(LinearVelocity vx, LinearVelocity vy, Rotation2d targetAngle) {}

  public final class PathFollower {
    private final CSPPath path;
    private final CSPConstraints baseConstraints;
    private final double lookaheadMeters;
    private final double trackingWindowMeters;
    private final double lateralGain;
    private final double startingSpeed;
    private final double endingSpeed;
    private double lastProgress = 0.0;

    private PathFollower(PathPlannerPath path, double startingSpeed, double endingSpeed) {
      this(path, seedFrom(path, null).constraints(), startingSpeed, endingSpeed);
    }

    private PathFollower(
        PathPlannerPath path,
        CSPConstraints baseConstraints,
        double startingSpeed,
        double endingSpeed) {
      this.path = CSPPath.fromPathPlannerPath(Objects.requireNonNull(path, "path cannot be null"));
      this.baseConstraints =
          Objects.requireNonNull(baseConstraints, "baseConstraints cannot be null");
      this.lookaheadMeters = 0.35;
      this.trackingWindowMeters = 0.75;
      this.lateralGain = 3.5;
      this.startingSpeed = Math.max(0.0, startingSpeed);
      this.endingSpeed = Math.max(0.0, endingSpeed);
    }

    public void reset() {
      lastProgress = 0.0;
    }

    public CSPResult update(Pose2d current, ChassisSpeeds robotRelativeSpeeds) {
      double totalLength = path.getTotalLength();
      if (totalLength <= 1e-9) {
        return new CSPResult(MetersPerSecond.of(0), MetersPerSecond.of(0), current.getRotation());
      }

      double progress = path.closestProgress(current, lastProgress, trackingWindowMeters);
      progress = Math.max(progress, lastProgress);
      lastProgress = progress;

      double progressArc = path.arcLengthAt(progress);
      double remainingMeters = Math.max(0.0, totalLength - progressArc);

      double targetArc = Math.min(totalLength, progressArc + lookaheadMeters);
      double targetProgress = targetArc / totalLength;

      Pose2d translationGoal = path.sample(targetProgress);
      Rotation2d targetRotation = path.rotationAt(targetProgress);
      targetRotation = nearestEquivalent(current.getRotation(), targetRotation);

      Pose2d goal = new Pose2d(translationGoal.getTranslation(), targetRotation);

      CSPConstraints activeConstraints =
          constraintsAtProgress(path.getSourcePath(), targetProgress);

      double interpolatedSpeed = interpolateSpeed(startingSpeed, endingSpeed, targetProgress);
      double brakeLimitedSpeed =
          maxSpeedToReachEndSpeed(
              Math.max(0.0, totalLength - progressArc),
              endingSpeed,
              activeConstraints.acceleration);

      double pathSpeedCap = Math.min(interpolatedSpeed, brakeLimitedSpeed);
      if (Double.isFinite(activeConstraints.velocity)) {
        pathSpeedCap = Math.min(pathSpeedCap, activeConstraints.velocity);
      }

      if (remainingMeters <= Math.max(0.50, lookaheadMeters * 2.0)) {
        pathSpeedCap = Math.min(pathSpeedCap, Math.max(endingSpeed, 0.35));
      }

      pathSpeedCap = Math.max(0.0, pathSpeedCap);

      CSPResult raw =
          calculate(
              current,
              robotRelativeSpeeds,
              new CSPTarget(goal).withVelocity(pathSpeedCap),
              activeConstraints,
              pathSpeedCap);

      Translation2d desiredFieldVelocity =
          new Translation2d(raw.vx().in(MetersPerSecond), raw.vy().in(MetersPerSecond));

      Translation2d pathTangent = path.tangentAt(targetProgress);
      Translation2d toGoal = goal.getTranslation().minus(current.getTranslation());

      double alongAmount = toGoal.getX() * pathTangent.getX() + toGoal.getY() * pathTangent.getY();
      Translation2d alongComponent = pathTangent.times(alongAmount);
      Translation2d lateralError = toGoal.minus(alongComponent);
      Translation2d correction = lateralError.times(lateralGain);

      desiredFieldVelocity = desiredFieldVelocity.plus(correction);

      double finalCap = activeConstraints.velocity;
      if (Double.isFinite(pathSpeedCap)) {
        finalCap = Math.min(finalCap, Math.max(0.0, pathSpeedCap));
      }

      double mag = desiredFieldVelocity.getNorm();
      if (mag > finalCap && mag > 1e-9) {
        desiredFieldVelocity = desiredFieldVelocity.times(finalCap / mag);
      }

      return new CSPResult(
          MetersPerSecond.of(desiredFieldVelocity.getX()),
          MetersPerSecond.of(desiredFieldVelocity.getY()),
          targetRotation);
    }

    private CSPConstraints constraintsAtProgress(PathPlannerPath sourcePath, double progress) {
      CSPConstraints base = baseConstraints;
      List<ConstraintsZone> zones = sourcePath.getConstraintZones();

      if (zones == null || zones.isEmpty()) {
        return base;
      }

      List<?> waypoints = sourcePath.getWaypoints();
      int waypointCount = waypoints == null ? 0 : waypoints.size();
      if (waypointCount < 2) {
        return base;
      }

      double waypointSpan = Math.max(1.0, waypointCount - 1.0);
      double zoneProgress = clamp01(progress);

      for (ConstraintsZone zone : zones) {
        double zoneMin = zone.minPosition() / waypointSpan;
        double zoneMax = zone.maxPosition() / waypointSpan;

        if (zoneProgress >= zoneMin && zoneProgress <= zoneMax) {
          PathConstraints pc = zone.constraints();
          return new CSPConstraints(
              Math.max(base.velocity, pc.maxVelocity().in(MetersPerSecond)), 1000, 1000);
        }
      }

      return base;
    }

    public boolean isFinished(Pose2d current) {
      Pose2d end = path.samplePose(1.0);

      double xyTol = Math.max(profile.getErrorXY().in(Meters), 0.07);
      double thetaTol = Math.max(profile.getErrorTheta().in(Radians), Math.toRadians(5));

      double xyError = current.getTranslation().getDistance(end.getTranslation());
      double thetaError =
          Math.abs(normalizeRadians(current.getRotation().minus(end.getRotation()).getRadians()));

      return xyError <= xyTol && thetaError <= thetaTol;
    }

    private double interpolateSpeed(double start, double end, double t) {
      t = clamp01(t);
      return Math.max(0.0, start + (end - start) * t);
    }

    private double maxSpeedToReachEndSpeed(double remainingMeters, double endSpeed, double accel) {
      if (remainingMeters <= 1e-9) {
        return endSpeed;
      }
      if (!(accel > 1e-9) || Double.isInfinite(accel)) {
        return Double.POSITIVE_INFINITY;
      }
      return Math.sqrt(Math.max(0.0, endSpeed * endSpeed + 2.0 * accel * remainingMeters));
    }
  }

  public static class CSPConstraints {
    protected double velocity;
    protected double acceleration;
    protected double jerk;
    protected final double x0;
    protected final double v0;

    public CSPConstraints() {
      this(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public CSPConstraints(double velocity, double acceleration, double jerk) {
      this.velocity = velocity;
      this.acceleration = acceleration;
      this.jerk = jerk;

      if (Double.isFinite(acceleration)
          && Double.isFinite(jerk)
          && acceleration > 1e-9
          && jerk > 1e-9) {
        x0 = Math.pow(acceleration, 3.0) / (18.0 * jerk * jerk);
        v0 = jerkConstrainedVelocity(x0);
      } else {
        x0 = 0.0;
        v0 = 0.0;
      }
    }

    public CSPConstraints(double acceleration, double jerk) {
      this(Double.POSITIVE_INFINITY, acceleration, jerk);
    }

    public CSPConstraints withVelocity(double newVelocity) {
      return new CSPConstraints(newVelocity, this.acceleration, this.jerk);
    }

    public CSPConstraints withAcceleration(double newAcceleration) {
      return new CSPConstraints(this.velocity, newAcceleration, this.jerk);
    }

    public CSPConstraints withJerk(double newJerk) {
      return new CSPConstraints(this.velocity, this.acceleration, newJerk);
    }

    protected double calculateMaxVelocity(double dist) {
      double max;
      if (dist > x0) {
        max = accelerationConstrainedVelocity(dist);
      } else {
        max = jerkConstrainedVelocity(dist);
      }
      return Math.min(max, velocity);
    }

    private double accelerationConstrainedVelocity(double dist) {
      return Math.sqrt(v0 * v0 + 2.0 * acceleration * (dist - x0));
    }

    private double jerkConstrainedVelocity(double dist) {
      return Math.pow((4.5 * Math.pow(dist, 2.0)) * jerk, 1.0 / 3.0);
    }
  }

  public static class CSPProfile {
    protected CSPConstraints constraints;
    protected Distance errorXY;
    protected Angle errorTheta;
    protected Distance beelineRadius;

    public CSPProfile(CSPConstraints constraints) {
      this.constraints = constraints;
      errorXY = Meters.of(0);
      errorTheta = Rotations.of(0);
      beelineRadius = Meters.of(0);
    }

    public CSPProfile withErrorXY(Distance errorXY) {
      this.errorXY = errorXY;
      return this;
    }

    public CSPProfile withErrorTheta(Angle errorTheta) {
      this.errorTheta = errorTheta;
      return this;
    }

    public CSPProfile withConstraints(CSPConstraints constraints) {
      this.constraints = constraints;
      return this;
    }

    public CSPProfile withBeelineRadius(Distance beelineRadius) {
      this.beelineRadius = beelineRadius;
      return this;
    }

    public Distance getErrorXY() {
      return errorXY;
    }

    public Angle getErrorTheta() {
      return errorTheta;
    }

    public CSPConstraints getConstraints() {
      return constraints;
    }

    public Distance getBeelineRadius() {
      return beelineRadius;
    }
  }

  public static class CSPTarget {
    protected Pose2d reference;
    protected Optional<Rotation2d> entryAngle;
    protected double velocity;
    protected Optional<Distance> rotationRadius;

    public CSPTarget(Pose2d pose) {
      this.reference = pose;
      this.velocity = 0;
      this.entryAngle = Optional.empty();
      this.rotationRadius = Optional.empty();
    }

    public CSPTarget withReference(Pose2d reference) {
      CSPTarget target = this.clone();
      target.reference = reference;
      return target;
    }

    public CSPTarget withEntryAngle(Rotation2d entryAngle) {
      CSPTarget target = this.clone();
      target.entryAngle = Optional.of(entryAngle);
      return target;
    }

    public CSPTarget withVelocity(double velocity) {
      CSPTarget target = this.clone();
      target.velocity = velocity;
      return target;
    }

    public CSPTarget withRotationRadius(Distance radius) {
      CSPTarget copy = this.clone();
      copy.rotationRadius = Optional.of(radius);
      return copy;
    }

    public Pose2d getReference() {
      return reference;
    }

    public Optional<Rotation2d> getEntryAngle() {
      return entryAngle;
    }

    public double getVelocity() {
      return velocity;
    }

    public Optional<Distance> getRotationRadius() {
      return rotationRadius;
    }

    public CSPTarget clone() {
      CSPTarget target = new CSPTarget(reference);
      target.velocity = velocity;
      target.entryAngle = entryAngle;
      target.rotationRadius = rotationRadius;
      return target;
    }

    public CSPTarget withoutEntryAngle() {
      CSPTarget target = new CSPTarget(reference);
      target.velocity = velocity;
      target.rotationRadius = rotationRadius;
      return target;
    }
  }

  public static class CSPPath {
    private final PathPlannerPath source;
    private final List<Pose2d> poses;
    private final double[] cumulativeDistances;
    private final double[] curvatures;
    private final double totalLength;
    private final List<RotationSample> rotationSamples;

    public static CSPPath fromPathPlannerPath(PathPlannerPath path) {
      return new CSPPath(path, 0, poses -> poses);
    }

    public static CSPPath fromPathPlannerPath(
        PathPlannerPath path, UnaryOperator<List<Pose2d>> manipulator) {
      return new CSPPath(path, 0, manipulator);
    }

    public static CSPPath fromPathPlannerPath(
        PathPlannerPath path, int resampleCount, UnaryOperator<List<Pose2d>> manipulator) {
      return new CSPPath(path, resampleCount, manipulator);
    }

    private CSPPath(
        PathPlannerPath source, int resampleCount, UnaryOperator<List<Pose2d>> manipulator) {
      this.source = Objects.requireNonNull(source, "source cannot be null");

      List<Pose2d> base = new ArrayList<>(source.getPathPoses());
      if (base.size() < 2) {
        throw new IllegalArgumentException("PathPlannerPath must contain at least 2 poses");
      }

      List<Pose2d> working = base;
      if (resampleCount >= 2) {
        CSPPath temp = new CSPPath(source, base);
        List<Pose2d> resampled = new ArrayList<>(resampleCount);
        for (int i = 0; i < resampleCount; i++) {
          double s = i / (double) (resampleCount - 1);
          resampled.add(temp.sample(s));
        }
        working = resampled;
      }

      List<Pose2d> manipulated = manipulator.apply(List.copyOf(working));
      if (manipulated == null || manipulated.size() < 2) {
        throw new IllegalArgumentException("Manipulated path must contain at least 2 poses");
      }

      this.poses = List.copyOf(manipulated);
      this.cumulativeDistances = buildCumulativeDistances(this.poses);
      this.curvatures = buildCurvatures(this.poses);
      this.totalLength = cumulativeDistances[cumulativeDistances.length - 1];
      this.rotationSamples = buildRotationSamplesFromPathPoints(source, this.totalLength);
    }

    private CSPPath(PathPlannerPath source, List<Pose2d> poses) {
      this.source = Objects.requireNonNull(source, "source cannot be null");
      if (poses == null || poses.size() < 2) {
        throw new IllegalArgumentException("poses must contain at least 2 entries");
      }
      this.poses = List.copyOf(poses);
      this.cumulativeDistances = buildCumulativeDistances(this.poses);
      this.curvatures = buildCurvatures(this.poses);
      this.totalLength = cumulativeDistances[cumulativeDistances.length - 1];
      this.rotationSamples = buildRotationSamplesFromPathPoints(source, this.totalLength);
    }

    public PathPlannerPath getSourcePath() {
      return source;
    }

    public List<Pose2d> getPoses() {
      return poses;
    }

    public double getTotalLength() {
      return totalLength;
    }

    public Pose2d sample(double s) {
      if (poses.size() == 1 || totalLength <= 1e-9) {
        return poses.get(0);
      }

      s = clamp01(s);
      double targetDist = s * totalLength;

      if (targetDist <= 0.0) {
        return poses.get(0);
      }
      if (targetDist >= totalLength) {
        return poses.get(poses.size() - 1);
      }

      int i = 0;
      while (i < cumulativeDistances.length - 2 && cumulativeDistances[i + 1] < targetDist) {
        i++;
      }

      double d1 = cumulativeDistances[i];
      double d2 = cumulativeDistances[i + 1];
      double localT = (targetDist - d1) / Math.max(1e-9, (d2 - d1));

      return lerpPose(poses.get(i), poses.get(i + 1), localT);
    }

    public Pose2d samplePose(double s) {
      Pose2d p = sample(s);
      return new Pose2d(p.getTranslation(), rotationAt(s));
    }

    public double arcLengthAt(double s) {
      s = clamp01(s);
      return s * totalLength;
    }

    public double curvatureAt(double s) {
      if (poses.size() < 3) {
        return 0.0;
      }

      s = clamp01(s);
      if (s <= 0.0) {
        return curvatures[0];
      }
      if (s >= 1.0) {
        return curvatures[curvatures.length - 1];
      }

      double targetDist = s * totalLength;
      int i = 0;
      while (i < cumulativeDistances.length - 2 && cumulativeDistances[i + 1] < targetDist) {
        i++;
      }

      double d1 = cumulativeDistances[i];
      double d2 = cumulativeDistances[i + 1];
      double t = (targetDist - d1) / Math.max(1e-9, (d2 - d1));

      return curvatures[i] + (curvatures[i + 1] - curvatures[i]) * t;
    }

    public double maxSpeedFromCurvature(double s, double maxLateralAccel) {
      double kappa = Math.abs(curvatureAt(s));
      if (kappa < 1e-9 || Double.isInfinite(maxLateralAccel)) {
        return Double.POSITIVE_INFINITY;
      }
      return Math.sqrt(Math.max(0.0, maxLateralAccel / kappa));
    }

    public Rotation2d rotationAt(double s) {
      if (rotationSamples.isEmpty()) {
        return sample(s).getRotation();
      }

      s = clamp01(s);

      if (rotationSamples.size() == 1) {
        return rotationSamples.get(0).rotation();
      }
      if (s <= rotationSamples.get(0).s()) {
        return rotationSamples.get(0).rotation();
      }
      if (s >= rotationSamples.get(rotationSamples.size() - 1).s()) {
        return rotationSamples.get(rotationSamples.size() - 1).rotation();
      }

      int i = 0;
      while (i < rotationSamples.size() - 2 && rotationSamples.get(i + 1).s() < s) {
        i++;
      }

      RotationSample a = rotationSamples.get(i);
      RotationSample b = rotationSamples.get(i + 1);
      double t = (s - a.s()) / Math.max(1e-9, b.s() - a.s());
      double smooth = (1.0 - Math.cos(t * Math.PI)) / 2.0;

      double aRad = a.rotation().getRadians();
      double delta = normalizeRadians(b.rotation().getRadians() - aRad);

      return new Rotation2d(normalizeRadians(aRad + delta * smooth));
    }

    public Translation2d tangentAt(double s) {
      double eps = Math.max(0.01, 1.0 / Math.max(40.0, poses.size() * 2.0));
      double s0 = clamp01(s - eps);
      double s1 = clamp01(s + eps);

      Translation2d a = sample(s0).getTranslation();
      Translation2d b = sample(s1).getTranslation();
      Translation2d d = b.minus(a);
      double n = d.getNorm();

      if (n < 1e-9) {
        return new Translation2d(1.0, 0.0);
      }
      return d.div(n);
    }

    public double closestProgress(Pose2d current) {
      return closestProgress(current, 0.0, Double.POSITIVE_INFINITY);
    }

    public double closestProgress(Pose2d current, double hintProgress, double searchWindowMeters) {
      if (totalLength <= 1e-9) {
        return 0.0;
      }

      hintProgress = clamp01(hintProgress);
      searchWindowMeters = Math.max(0.0, searchWindowMeters);

      double hintArc = hintProgress * totalLength;
      double minArc = Math.max(0.0, hintArc - searchWindowMeters);
      double maxArc = Math.min(totalLength, hintArc + searchWindowMeters);

      Translation2d p = current.getTranslation();

      double bestWindowScore = Double.POSITIVE_INFINITY;
      double bestWindowArc = hintArc;
      double bestAnyScore = Double.POSITIVE_INFINITY;
      double bestAnyArc = 0.0;

      for (int i = 0; i < poses.size() - 1; i++) {
        Translation2d a = poses.get(i).getTranslation();
        Translation2d b = poses.get(i + 1).getTranslation();

        double abx = b.getX() - a.getX();
        double aby = b.getY() - a.getY();
        double len2 = abx * abx + aby * aby;
        if (len2 < 1e-12) {
          continue;
        }

        double apx = p.getX() - a.getX();
        double apy = p.getY() - a.getY();
        double t = (apx * abx + apy * aby) / len2;
        t = Math.max(0.0, Math.min(1.0, t));

        double projX = a.getX() + abx * t;
        double projY = a.getY() + aby * t;

        double dx = p.getX() - projX;
        double dy = p.getY() - projY;
        double dist2 = dx * dx + dy * dy;

        double segLen = Math.sqrt(len2);
        double candidateArc = cumulativeDistances[i] + segLen * t;

        double branchPenalty = candidateArc - hintArc;
        double score = dist2 + 0.20 * branchPenalty * branchPenalty;

        if (score < bestAnyScore) {
          bestAnyScore = score;
          bestAnyArc = candidateArc;
        }

        if (candidateArc >= minArc && candidateArc <= maxArc && score < bestWindowScore) {
          bestWindowScore = score;
          bestWindowArc = candidateArc;
        }
      }

      double chosenArc = Double.isFinite(bestWindowScore) ? bestWindowArc : bestAnyArc;
      return clamp01(chosenArc / totalLength);
    }

    public PathPlannerPath toPathPlannerPath() {
      PathPlannerPath rebuilt =
          new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(poses),
              source.getGlobalConstraints(),
              source.getIdealStartingState(),
              source.getGoalEndState(),
              source.isReversed());

      rebuilt.name = source.name;
      rebuilt.preventFlipping = source.preventFlipping;
      return rebuilt;
    }

    private static double[] buildCumulativeDistances(List<Pose2d> poses) {
      double[] distances = new double[poses.size()];
      distances[0] = 0.0;
      for (int i = 1; i < poses.size(); i++) {
        double d = poses.get(i).getTranslation().getDistance(poses.get(i - 1).getTranslation());
        distances[i] = distances[i - 1] + d;
      }
      return distances;
    }

    private static double[] buildCurvatures(List<Pose2d> poses) {
      int n = poses.size();
      double[] k = new double[n];
      if (n < 3) {
        return k;
      }

      for (int i = 1; i < n - 1; i++) {
        Translation2d p0 = poses.get(i - 1).getTranslation();
        Translation2d p1 = poses.get(i).getTranslation();
        Translation2d p2 = poses.get(i + 1).getTranslation();

        double a = p0.getDistance(p1);
        double b = p1.getDistance(p2);
        double c = p0.getDistance(p2);

        double cross =
            cross(
                p1.getX() - p0.getX(),
                p1.getY() - p0.getY(),
                p2.getX() - p0.getX(),
                p2.getY() - p0.getY());

        double denom = a * b * c;
        if (denom < 1e-12) {
          k[i] = 0.0;
        } else {
          k[i] = (2.0 * cross) / denom;
        }
      }

      k[0] = k[1];
      k[n - 1] = k[n - 2];
      return k;
    }

    private static List<RotationSample> buildRotationSamplesFromPathPoints(
        PathPlannerPath source, double totalLength) {
      List<RotationSample> out = new ArrayList<>();
      List<PathPoint> points = source.getAllPathPoints();

      if (points == null || points.isEmpty()) {
        return List.of();
      }

      for (PathPoint point : points) {
        if (point.rotationTarget == null) {
          continue;
        }
        double s = totalLength > 1e-9 ? point.distanceAlongPath / totalLength : 0.0;
        out.add(new RotationSample(clamp01(s), point.rotationTarget.rotation()));
      }

      out.sort((a, b) -> Double.compare(a.s(), b.s()));
      return List.copyOf(out);
    }

    private static double cross(double ax, double ay, double bx, double by) {
      return ax * by - ay * bx;
    }

    private static Pose2d lerpPose(Pose2d a, Pose2d b, double t) {
      t = Math.max(0.0, Math.min(1.0, t));
      double x = a.getX() + (b.getX() - a.getX()) * t;
      double y = a.getY() + (b.getY() - a.getY()) * t;

      double aRad = a.getRotation().getRadians();
      double bRad = b.getRotation().getRadians();
      double delta = normalizeRadians(bRad - aRad);
      double rot = normalizeRadians(aRad + delta * t);

      return new Pose2d(new Translation2d(x, y), new Rotation2d(rot));
    }

    private record RotationSample(double s, Rotation2d rotation) {}
  }

  private static Rotation2d nearestEquivalent(Rotation2d reference, Rotation2d target) {
    double ref = reference.getRadians();
    double tgt = target.getRadians();
    return new Rotation2d(ref + normalizeRadians(tgt - ref));
  }

  private static double clamp01(double v) {
    return Math.max(0.0, Math.min(1.0, v));
  }

  private static double normalizeRadians(double radians) {
    double out = radians;
    while (out > Math.PI) {
      out -= 2.0 * Math.PI;
    }
    while (out < -Math.PI) {
      out += 2.0 * Math.PI;
    }
    return out;
  }
}
