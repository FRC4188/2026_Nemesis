package frc.robot.CSPLib.csppathing;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * KalmanPathing is a PathPlanner-based auto command factory that estimates path progress with a
 * small Kalman filter and uses that filtered progress to sample a PathPlanner trajectory.
 *
 * <p>The public factory takes a PathPlannerPath and the inputs required to generate a trajectory
 * from it, then creates the trajectory internally.
 */
public final class KalmanPathing {
  private static final double DEFAULT_SAMPLE_DT_SEC = 0.02;
  private static final double MIN_DT_SEC = 1e-3;
  private static final double DEFAULT_PROCESS_ACCEL_STD = 1.25;
  private static final double DEFAULT_MEASUREMENT_STD = 0.10;
  private static final double END_TOLERANCE_M = 0.05;

  private KalmanPathing() {}

  public static Command create(
      PathPlannerPath path,
      ChassisSpeeds startingSpeeds,
      Rotation2d startingRotation,
      RobotConfig robotConfig,
      PPHolonomicDriveController controller,
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> outputConsumer,
      Subsystem... requirements) {
    return create(
        path,
        startingSpeeds,
        startingRotation,
        robotConfig,
        controller,
        poseSupplier,
        outputConsumer,
        DEFAULT_PROCESS_ACCEL_STD,
        DEFAULT_MEASUREMENT_STD,
        requirements);
  }

  public static Command create(
      PathPlannerPath path,
      ChassisSpeeds startingSpeeds,
      Rotation2d startingRotation,
      RobotConfig robotConfig,
      PPHolonomicDriveController controller,
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> outputConsumer,
      double processAccelStd,
      double measurementStd,
      Subsystem... requirements) {

    Objects.requireNonNull(path, "path");
    Objects.requireNonNull(startingSpeeds, "startingSpeeds");
    Objects.requireNonNull(startingRotation, "startingRotation");
    Objects.requireNonNull(robotConfig, "robotConfig");
    Objects.requireNonNull(controller, "controller");
    Objects.requireNonNull(poseSupplier, "poseSupplier");
    Objects.requireNonNull(outputConsumer, "outputConsumer");

    final PathPlannerTrajectory trajectory =
        path.generateTrajectory(startingSpeeds, startingRotation, robotConfig);
    final List<PathSample> samples = buildSamples(trajectory, DEFAULT_SAMPLE_DT_SEC);
    final double totalLengthMeters = samples.get(samples.size() - 1).arcLengthMeters;
    final ProgressKalmanFilter filter = new ProgressKalmanFilter(processAccelStd, measurementStd);

    class StateHolder {
      double lastTimeSec;
      double lastResetSec;
    }

    final StateHolder state = new StateHolder();

    return new FunctionalCommand(
        () -> {
          state.lastTimeSec = Timer.getFPGATimestamp();
          state.lastResetSec = state.lastTimeSec;

          Pose2d currentPose = poseSupplier.get();
          controller.reset(currentPose, startingSpeeds);

          double seed = clamp(projectProgress(currentPose, samples), 0.0, totalLengthMeters);
          filter.reset(seed, 0.0);
        },
        () -> {
          double now = Timer.getFPGATimestamp();
          double dt = Math.max(MIN_DT_SEC, now - state.lastTimeSec);
          state.lastTimeSec = now;

          Pose2d currentPose = poseSupplier.get();
          double measuredProgress =
              clamp(projectProgress(currentPose, samples), 0.0, totalLengthMeters);

          filter.predict(dt);

          double predictedProgress = filter.getPositionMeters();
          double innovation = Math.abs(measuredProgress - predictedProgress);
          boolean shouldReset =
              innovation > Math.max(0.5, 4.0 * measurementStd) && (now - state.lastResetSec) > 0.25;

          if (shouldReset) {
            filter.reset(measuredProgress, 0.0);
            state.lastResetSec = now;
          } else {
            filter.update(measuredProgress);
          }

          double filteredProgress = clamp(filter.getPositionMeters(), 0.0, totalLengthMeters);
          double sampledTimeSec = progressToTime(filteredProgress, samples, trajectory);
          PathPlannerTrajectoryState goal = trajectory.sample(sampledTimeSec);

          ChassisSpeeds speeds = controller.calculateRobotRelativeSpeeds(currentPose, goal);
          outputConsumer.accept(speeds);
        },
        interrupted -> outputConsumer.accept(new ChassisSpeeds()),
        () -> filter.getPositionMeters() >= totalLengthMeters - END_TOLERANCE_M,
        requirements);
  }

  public static PPHolonomicDriveController createDefaultHolonomicController() {
    return new PPHolonomicDriveController(
        new PIDConstants(2.0, 0.0, 0.0), new PIDConstants(2.0, 0.0, 0.0));
  }

  private static double projectProgress(Pose2d pose, List<PathSample> samples) {
    Translation2d p = pose.getTranslation();
    double bestDistSq = Double.POSITIVE_INFINITY;
    double bestS = 0.0;

    for (int i = 0; i < samples.size() - 1; i++) {
      PathSample a = samples.get(i);
      PathSample b = samples.get(i + 1);

      Translation2d aPos = a.pose.getTranslation();
      Translation2d bPos = b.pose.getTranslation();
      Translation2d ab = bPos.minus(aPos);
      Translation2d ap = p.minus(aPos);

      double abNormSq = ab.getNorm() * ab.getNorm();
      if (abNormSq < 1e-12) {
        continue;
      }

      double u = clamp(ap.dot(ab) / abNormSq, 0.0, 1.0);
      Translation2d projection = aPos.plus(ab.times(u));
      double distanceSq = projection.getDistance(p);
      distanceSq *= distanceSq;

      if (distanceSq < bestDistSq) {
        bestDistSq = distanceSq;
        bestS = a.arcLengthMeters + u * (b.arcLengthMeters - a.arcLengthMeters);
      }
    }

    return bestS;
  }

  private static double progressToTime(
      double progressMeters, List<PathSample> samples, PathPlannerTrajectory trajectory) {
    double totalLengthMeters = samples.get(samples.size() - 1).arcLengthMeters;
    if (progressMeters <= 0.0) {
      return 0.0;
    }
    if (progressMeters >= totalLengthMeters) {
      return trajectory.getTotalTimeSeconds();
    }

    int low = 0;
    int high = samples.size() - 1;
    while (low + 1 < high) {
      int mid = (low + high) >>> 1;
      if (samples.get(mid).arcLengthMeters < progressMeters) {
        low = mid;
      } else {
        high = mid;
      }
    }

    PathSample a = samples.get(low);
    PathSample b = samples.get(high);
    double span = b.arcLengthMeters - a.arcLengthMeters;
    if (Math.abs(span) < 1e-12) {
      return a.timeSeconds;
    }

    double u = (progressMeters - a.arcLengthMeters) / span;
    return a.timeSeconds + u * (b.timeSeconds - a.timeSeconds);
  }

  private static List<PathSample> buildSamples(PathPlannerTrajectory trajectory, double dt) {
    List<PathSample> out = new ArrayList<>();

    double totalTimeSec = trajectory.getTotalTimeSeconds();
    PathPlannerTrajectoryState previous = trajectory.sample(0.0);
    out.add(new PathSample(0.0, previous.pose, 0.0));

    double cumulativeLengthMeters = 0.0;
    for (double t = dt; t < totalTimeSec; t += dt) {
      PathPlannerTrajectoryState current = trajectory.sample(t);
      cumulativeLengthMeters +=
          current.pose.getTranslation().getDistance(previous.pose.getTranslation());
      out.add(new PathSample(t, current.pose, cumulativeLengthMeters));
      previous = current;
    }

    PathPlannerTrajectoryState end = trajectory.sample(totalTimeSec);
    cumulativeLengthMeters += end.pose.getTranslation().getDistance(previous.pose.getTranslation());
    out.add(new PathSample(totalTimeSec, end.pose, cumulativeLengthMeters));

    if (out.size() < 2) {
      throw new IllegalArgumentException("Trajectory must contain at least two states.");
    }

    return List.copyOf(out);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private record PathSample(double timeSeconds, Pose2d pose, double arcLengthMeters) {}

  /** Minimal 1D constant-velocity Kalman filter over path progress. */
  private static final class ProgressKalmanFilter {
    private double sMeters;
    private double sDotMetersPerSec;
    private double p00 = 1.0;
    private double p01 = 0.0;
    private double p10 = 0.0;
    private double p11 = 1.0;
    private final double accelStd;
    private final double measurementStd;

    ProgressKalmanFilter(double accelStd, double measurementStd) {
      this.accelStd = Math.max(1e-6, accelStd);
      this.measurementStd = Math.max(1e-6, measurementStd);
    }

    void reset(double positionMeters, double velocityMetersPerSec) {
      sMeters = positionMeters;
      sDotMetersPerSec = velocityMetersPerSec;
      p00 = 1.0;
      p01 = 0.0;
      p10 = 0.0;
      p11 = 1.0;
    }

    void predict(double dt) {
      sMeters += sDotMetersPerSec * dt;

      double a00 = 1.0;
      double a01 = dt;
      double a10 = 0.0;
      double a11 = 1.0;

      double np00 = a00 * p00 + a01 * p10;
      double np01 = a00 * p01 + a01 * p11;
      double np10 = a10 * p00 + a11 * p10;
      double np11 = a10 * p01 + a11 * p11;

      double q = accelStd * accelStd;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;
      double dt4 = dt2 * dt2;
      double q00 = 0.25 * dt4 * q;
      double q01 = 0.5 * dt3 * q;
      double q10 = q01;
      double q11 = dt2 * q;

      p00 = np00 * a00 + np01 * a01 + q00;
      p01 = np00 * a10 + np01 * a11 + q01;
      p10 = np10 * a00 + np11 * a01 + q10;
      p11 = np10 * a10 + np11 * a11 + q11;
    }

    void update(double measuredPositionMeters) {
      double r = measurementStd * measurementStd;
      double innovation = measuredPositionMeters - sMeters;
      double sCov = p00 + r;
      double k0 = p00 / sCov;
      double k1 = p10 / sCov;

      sMeters += k0 * innovation;
      sDotMetersPerSec += k1 * innovation;

      double oldP00 = p00;
      double oldP01 = p01;
      double oldP10 = p10;
      double oldP11 = p11;

      p00 = (1.0 - k0) * oldP00;
      p01 = (1.0 - k0) * oldP01;
      p10 = oldP10 - k1 * oldP00;
      p11 = oldP11 - k1 * oldP01;
    }

    double getPositionMeters() {
      return sMeters;
    }
  }
}
