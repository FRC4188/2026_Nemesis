package frc.robot.CSPLib.csppathing;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

public final class CSPPath {
  private SubsystemBase driveSubsystem;
  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
  private Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;
  private PIDController translationController;
  private PIDController rotationController;
  private PIDController crossTrackController;
  private Path.DefaultGlobalConstraints defaultGlobalConstraints;

  private boolean configured = false;
  private boolean defaultReliable = false;

  private double minHandoffRadiusMeters = 0.08;
  private double maxHandoffRadiusMeters = 0.45;

  public CSPPath() {}

  public CSPPath withReliability(boolean reliable) {
    this.defaultReliable = reliable;
    return this;
  }

  public CSPPath withMinHandoffRadiusMeters(double meters) {
    this.minHandoffRadiusMeters = Math.max(0.0, meters);
    return this;
  }

  public CSPPath withMaxHandoffRadiusMeters(double meters) {
    this.maxHandoffRadiusMeters = Math.max(0.0, meters);
    return this;
  }

  public CSPPath configure(
      SubsystemBase driveSubsystem,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
      PIDController translationController,
      PIDController rotationController,
      PIDController crossTrackController,
      Path.DefaultGlobalConstraints defaultGlobalConstraints) {
    this.driveSubsystem = Objects.requireNonNull(driveSubsystem, "driveSubsystem");
    this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier");
    this.robotRelativeSpeedsSupplier =
        Objects.requireNonNull(robotRelativeSpeedsSupplier, "robotRelativeSpeedsSupplier");
    this.robotRelativeSpeedsConsumer =
        Objects.requireNonNull(robotRelativeSpeedsConsumer, "robotRelativeSpeedsConsumer");
    this.translationController =
        Objects.requireNonNull(translationController, "translationController");
    this.rotationController = Objects.requireNonNull(rotationController, "rotationController");
    this.crossTrackController =
        Objects.requireNonNull(crossTrackController, "crossTrackController");
    this.defaultGlobalConstraints =
        Objects.requireNonNull(defaultGlobalConstraints, "defaultGlobalConstraints");

    Path.setDefaultGlobalConstraints(defaultGlobalConstraints);
    configured = true;
    return this;
  }

  public Rotation2d getOptimalStartingModuleOrientation(PathPlannerPath pathPlannerPath) {
    Objects.requireNonNull(pathPlannerPath, "pathPlannerPath");
    return pathPlannerPath.getInitialHeading();
  }

  public Command build(PathPlannerPath pathPlannerPath) {
    return build(pathPlannerPath, defaultReliable);
  }

  public Command build(PathPlannerPath pathPlannerPath, boolean reliable) {
    ensureConfigured();
    FollowPath.Builder builder = createBuilder(reliable);
    return builder.build(convertToBLinePath(pathPlannerPath, reliable));
  }

  public Path convertToBLinePath(PathPlannerPath pathPlannerPath) {
    return convertToBLinePath(pathPlannerPath, defaultReliable);
  }

  public Path convertToBLinePath(PathPlannerPath pathPlannerPath, boolean reliable) {
    Objects.requireNonNull(pathPlannerPath, "pathPlannerPath");

    List<PathPoint> points = pathPlannerPath.getAllPathPoints();
    if (points == null || points.isEmpty()) {
      throw new IllegalArgumentException("PathPlannerPath contains no path points.");
    }

    Path.PathConstraints blineConstraints = buildBLineConstraints(pathPlannerPath);
    List<Path.PathElement> elements = buildBLineElements(pathPlannerPath, points, reliable);

    return new Path(elements, blineConstraints);
  }

  private FollowPath.Builder createBuilder(boolean reliable) {
    return new FollowPath.Builder(
            driveSubsystem,
            poseSupplier,
            robotRelativeSpeedsSupplier,
            robotRelativeSpeedsConsumer,
            translationController,
            rotationController,
            crossTrackController)
        .withDefaultShouldFlip()
        .withTRatioBasedTranslationHandoffs(reliable);
  }

  private Path.PathConstraints buildBLineConstraints(PathPlannerPath pathPlannerPath) {
    PathConstraints global = pathPlannerPath.getGlobalConstraints();

    List<Path.RangedConstraint> linearVelocityRanges = new ArrayList<>();
    List<Path.RangedConstraint> linearAccelerationRanges = new ArrayList<>();
    List<Path.RangedConstraint> angularVelocityRanges = new ArrayList<>();
    List<Path.RangedConstraint> angularAccelerationRanges = new ArrayList<>();

    for (ConstraintsZone zone : pathPlannerPath.getConstraintZones()) {
      PathConstraints z = zone.constraints();

      // These are kept in path-relative order by the zone's min/max positions.
      // If your BLine version expects a different ranged-constraint format, only
      // this section should need adjustment.
      linearVelocityRanges.add(
          new Path.RangedConstraint(
              z.maxVelocityMPS(), (int) zone.minPosition(), (int) zone.maxPosition()));
      linearAccelerationRanges.add(
          new Path.RangedConstraint(
              z.maxAccelerationMPSSq(), (int) zone.minPosition(), (int) zone.maxPosition()));
      angularVelocityRanges.add(
          new Path.RangedConstraint(
              Math.toDegrees(z.maxAngularVelocityRadPerSec()),
              (int) zone.minPosition(),
              (int) zone.maxPosition()));
      angularAccelerationRanges.add(
          new Path.RangedConstraint(
              Math.toDegrees(z.maxAngularAccelerationRadPerSecSq()),
              (int) zone.minPosition(),
              (int) zone.maxPosition()));
    }

    // Global fallback.
    linearVelocityRanges.add(
        new Path.RangedConstraint(global.maxVelocityMPS(), 0, Integer.MAX_VALUE));
    linearAccelerationRanges.add(
        new Path.RangedConstraint(global.maxAccelerationMPSSq(), 0, Integer.MAX_VALUE));
    angularVelocityRanges.add(
        new Path.RangedConstraint(
            Math.toDegrees(global.maxAngularVelocityRadPerSec()), 0, Integer.MAX_VALUE));
    angularAccelerationRanges.add(
        new Path.RangedConstraint(
            Math.toDegrees(global.maxAngularAccelerationRadPerSecSq()), 0, Integer.MAX_VALUE));

    return new Path.PathConstraints()
        .setMaxVelocityMetersPerSec(linearVelocityRanges.toArray(new Path.RangedConstraint[0]))
        .setMaxAccelerationMetersPerSec2(
            linearAccelerationRanges.toArray(new Path.RangedConstraint[0]))
        .setMaxVelocityDegPerSec(angularVelocityRanges.toArray(new Path.RangedConstraint[0]))
        .setMaxAccelerationDegPerSec2(
            angularAccelerationRanges.toArray(new Path.RangedConstraint[0]))
        .setEndTranslationToleranceMeters(0.03)
        .setEndRotationToleranceDeg(2.0);
  }

  private List<Path.PathElement> buildBLineElements(
      PathPlannerPath pathPlannerPath, List<PathPoint> points, boolean reliable) {
    List<OrderedElement> ordered = new ArrayList<>();

    IdealStartingState idealStart = pathPlannerPath.getIdealStartingState();
    GoalEndState goalEnd = pathPlannerPath.getGoalEndState();
    Rotation2d startRotation =
        (idealStart != null) ? idealStart.rotation() : pathPlannerPath.getInitialHeading();
    Rotation2d endRotation = goalEnd.rotation();

    for (int i = 0; i < points.size(); i++) {
      PathPoint p = points.get(i);
      Translation2d translation = p.position;

      double handoffRadius = computeBestHandoffRadius(points, i, reliable);

      if (i == 0) {
        ordered.add(
            new OrderedElement(
                progressOf(i, points.size()),
                0,
                new Path.Waypoint(translation, handoffRadius, startRotation, true)));
      } else if (i == points.size() - 1) {
        ordered.add(
            new OrderedElement(
                progressOf(i, points.size()),
                0,
                new Path.Waypoint(translation, handoffRadius, endRotation, true)));
      } else {
        ordered.add(
            new OrderedElement(
                progressOf(i, points.size()),
                0,
                new Path.TranslationTarget(translation.getX(), translation.getY(), handoffRadius)));
      }
    }

    for (RotationTarget rt : pathPlannerPath.getRotationTargets()) {
      ordered.add(
          new OrderedElement(
              rt.position(), 1, new Path.RotationTarget(rt.rotation(), rt.position(), true)));
    }

    ordered.sort(
        Comparator.comparingDouble(OrderedElement::position)
            .thenComparingInt(OrderedElement::priority));

    List<Path.PathElement> elements = new ArrayList<>(ordered.size());
    for (OrderedElement e : ordered) {
      elements.add(e.element());
    }
    return elements;
  }

  private double computeBestHandoffRadius(List<PathPoint> points, int index, boolean reliable) {
    if (points.size() <= 1) {
      return minHandoffRadiusMeters;
    }

    PathPoint current = points.get(index);

    double inLen =
        (index > 0)
            ? distance(points.get(index - 1), current)
            : distance(current, points.get(index + 1));

    double outLen = (index < points.size() - 1) ? distance(current, points.get(index + 1)) : inLen;

    double shorterSide = Math.min(inLen, outLen);
    double longerSide = Math.max(inLen, outLen);

    double turnAngle = 0.0;
    if (index > 0 && index < points.size() - 1) {
      Translation2d prevVec = current.position.minus(points.get(index - 1).position);
      Translation2d nextVec = points.get(index + 1).position.minus(current.position);
      turnAngle = angleBetween(prevVec, nextVec);
    }

    // 1.0 = straight, 0.0 = very sharp
    double straightness = 1.0 - clamp(turnAngle / Math.PI, 0.0, 1.0);

    // 1.0 = balanced neighbors, 0.0 = one side is much shorter
    double balance = (longerSide <= 1e-9) ? 0.0 : clamp(shorterSide / longerSide, 0.0, 1.0);

    // Geometry-driven base radius.
    double baseRadius = shorterSide * (0.10 + 0.28 * straightness + 0.10 * balance);

    // Reliability mode makes the robot hand off earlier and more conservatively.
    double reliabilityMultiplier =
        reliable ? (1.20 + 0.10 * straightness + 0.05 * balance) : (1.00 + 0.03 * straightness);

    // End-of-path gets a little more forgiving in reliable mode.
    double endProgress = progressOf(index, points.size());
    double endProximity = smoothstep(0.72, 1.0, endProgress);
    double endMultiplier = reliable ? (1.0 + 0.15 * endProximity) : 1.0;

    double radius = baseRadius * reliabilityMultiplier * endMultiplier;

    // Hard cap relative to local spacing.
    radius = Math.min(radius, shorterSide * (reliable ? 0.50 : 0.42));

    // Safety clamp.
    radius = clamp(radius, minHandoffRadiusMeters, maxHandoffRadiusMeters);

    return radius;
  }

  private double distance(PathPoint a, PathPoint b) {
    return a.position.getDistance(b.position);
  }

  private double angleBetween(Translation2d a, Translation2d b) {
    double ax = a.getX();
    double ay = a.getY();
    double bx = b.getX();
    double by = b.getY();

    double magA = Math.hypot(ax, ay);
    double magB = Math.hypot(bx, by);
    if (magA < 1e-9 || magB < 1e-9) {
      return 0.0;
    }

    double cos = (ax * bx + ay * by) / (magA * magB);
    cos = clamp(cos, -1.0, 1.0);
    return Math.acos(cos);
  }

  private double progressOf(int index, int size) {
    if (size <= 1) {
      return 0.0;
    }
    return index / (double) (size - 1);
  }

  private double smoothstep(double edge0, double edge1, double x) {
    if (edge0 == edge1) {
      return x >= edge1 ? 1.0 : 0.0;
    }
    double t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private void ensureConfigured() {
    if (!configured) {
      throw new IllegalStateException("CSPPath must be configured before building commands.");
    }
  }

  private record OrderedElement(double position, int priority, Path.PathElement element) {}
}
