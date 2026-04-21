package frc.robot.CSPLib.csppathing;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

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
 * A pose-parameterized motion controller, switching time to pose parameterization Best to be used
 * with CSPPathing
 */

/*
 * Basically a fork of Autopilot, just CSP style (4188 RAHHHH)
 */
public class CSPPilot {
  private final CSPProfile profile;
  private final double dt = 0.020;

  public CSPPilot(CSPProfile profile) {
    this.profile = Objects.requireNonNull(profile, "profile cannot be null");
  }

  public CSPProfile getProfile() {
    return profile;
  }

  // calculates the resulting chassis speeds necessary at that point in space
  public CSPResult calculate(Pose2d current, ChassisSpeeds robotRelativeSpeeds, CSPTarget target) {

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

    if (target.getEntryAngle().isEmpty() || disp < profile.getBeelineRadius().in(Meters)) {
      Translation2d towardsTarget = offset.div(disp);
      Translation2d goal =
          towardsTarget.times(
              profile.getConstraints().calculateMaxVelocity(disp) + target.getVelocity());

      Translation2d out = correct(initial, goal);
      Translation2d velo = toGlobalCoordinateFrame(out, target);
      Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);
      return new CSPResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
    }

    double speed = profile.getConstraints().calculateMaxVelocity(disp) + target.getVelocity();
    Translation2d goal = calculateSwirlyVelocity(offset, speed);

    Translation2d out = correct(initial, goal);
    Translation2d velo = toGlobalCoordinateFrame(out, target);
    Rotation2d rot = getRotationTarget(current.getRotation(), target, disp);
    return new CSPResult(MetersPerSecond.of(velo.getX()), MetersPerSecond.of(velo.getY()), rot);
  }

  // makes it concise, i just didn't want to call pathfollewer like that
  public PathFollower followPath(PathPlannerPath path) {
    return new PathFollower(path);
  }

  // Basically the reciprocal of withrobotrelativespeeds, it is in the target frame
  private Translation2d toTargetCoordinateFrame(Translation2d coords, CSPTarget target) {
    Rotation2d entryAngle = target.getEntryAngle().orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle.unaryMinus());
  }

  // Turns a translation from the target-relative frame back into the field coords
  private Translation2d toGlobalCoordinateFrame(Translation2d coords, CSPTarget target) {
    Rotation2d entryAngle = target.getEntryAngle().orElse(Rotation2d.kZero);
    return coords.rotateBy(entryAngle);
  }

  // manages the max acceleration and max velocity to change the vectors
  private Translation2d correct(Translation2d initial, Translation2d goal) {
    Rotation2d angleOffset = Rotation2d.kZero;
    if (goal.getNorm() > 1e-9) {
      angleOffset = new Rotation2d(goal.getX(), goal.getY());
    }

    Translation2d adjustedGoal = goal.rotateBy(angleOffset.unaryMinus());
    Translation2d adjustedInitial = initial.rotateBy(angleOffset.unaryMinus());

    double initialI = adjustedInitial.getX();
    double goalI = adjustedGoal.getX();

    if (goalI > profile.getConstraints().velocity) {
      goalI = profile.getConstraints().velocity;
    }

    double adjustedI =
        Math.min(goalI, push(initialI, goalI, profile.getConstraints().acceleration));
    return new Translation2d(adjustedI, 0).rotateBy(angleOffset);
  }

  /** Uses the provided acceleration to step the start value toward the end value. */
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

  /** Swirly motion calculation, scaled by the desired target speed. */
  private Translation2d calculateSwirlyVelocity(Translation2d offset, double speed) {
    double disp = offset.getNorm();
    Rotation2d theta = new Rotation2d(offset.getX(), offset.getY());
    double rads = theta.getRadians();
    double dist = calculateSwirlyLength(rads, disp);

    double vx = theta.getCos() - rads * theta.getSin();
    double vy = rads * theta.getCos() + theta.getSin();

    double norm = Math.hypot(vx, vy);
    if (norm < 1e-9) {
      return new Translation2d(0, 0);
    }

    return new Translation2d(vx, vy)
        .div(norm)
        .times(Math.min(speed, profile.getConstraints().calculateMaxVelocity(dist)));
  }

  /** Precomputed integral used by the swirly motion model. */
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

  /**
   * If the robot is within the rotation radius, use the target rotation. Otherwise preserve the
   * current rotation.
   */
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

  /** Returns true if the robot is within tolerance of the target pose. */
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
    private final double lookaheadMeters;
    private final double trackingWindowMeters;
    private final double lateralGain;

    private double lastProgress = 0.0;

    private PathFollower(PathPlannerPath path) {
      this(path, 0.35, 0.75, 3.5);
    }

    private PathFollower(
        PathPlannerPath path,
        double lookaheadMeters,
        double trackingWindowMeters,
        double lateralGain) {
      this.path = CSPPath.fromPathPlannerPath(Objects.requireNonNull(path, "path cannot be null"));
      this.lookaheadMeters = Math.max(0.0, lookaheadMeters);
      this.trackingWindowMeters = Math.max(0.0, trackingWindowMeters);
      this.lateralGain = Math.max(0.0, lateralGain);
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
      double targetArc = Math.min(totalLength, progressArc + lookaheadMeters);
      double targetProgress = targetArc / totalLength;

      Pose2d translationGoal = path.sample(targetProgress);

      Rotation2d targetRotation = path.rotationAt(progress);
      targetRotation = nearestEquivalent(current.getRotation(), targetRotation);

      Pose2d goal = new Pose2d(translationGoal.getTranslation(), targetRotation);
      CSPResult raw = calculate(current, robotRelativeSpeeds, new CSPTarget(goal));

      Translation2d desiredFieldVelocity =
          new Translation2d(raw.vx().in(MetersPerSecond), raw.vy().in(MetersPerSecond));

      Translation2d pathTangent = path.tangentAt(targetProgress);
      Translation2d toGoal = goal.getTranslation().minus(current.getTranslation());

      double alongAmount = toGoal.getX() * pathTangent.getX() + toGoal.getY() * pathTangent.getY();
      Translation2d alongComponent = pathTangent.times(alongAmount);
      Translation2d lateralError = toGoal.minus(alongComponent);

      Translation2d correction = lateralError.times(lateralGain);
      desiredFieldVelocity = desiredFieldVelocity.plus(correction);

      return new CSPResult(
          MetersPerSecond.of(desiredFieldVelocity.getX()),
          MetersPerSecond.of(desiredFieldVelocity.getY()),
          targetRotation);
    }

    public boolean isFinished(Pose2d current) {
      double progress = path.closestProgress(current, lastProgress, trackingWindowMeters);
      Pose2d end = path.samplePose(1.0);
      return progress >= 0.999 && atTarget(current, new CSPTarget(end));
    }
  }

  // Motion constraints.
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
      x0 = Math.pow(acceleration, 3.0) / (18.0 * jerk * jerk);
      v0 = jerkConstrainedVelocity(x0);
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
      if (dist > x0) {
        return accelerationConstrainedVelocity(dist);
      }
      return jerkConstrainedVelocity(dist);
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

    public List<Pose2d> getPoses() {
      return poses;
    }

    public double getTotalLength() {
      return totalLength;
    }

    /** Sample the path by normalized arc length in [0, 1]. */
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

    // lerp.
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
