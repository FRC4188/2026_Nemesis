package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static Vision instance;

  public static synchronized Vision getInstance() {
    if (instance == null) {
      instance =
          switch (Constants.Robot.currentMode) {
            case REAL ->
                new Vision(
                    Drive.getInstance()::accept,
                    new VisionIOPhoton(
                        VisConstants.frontPho,
                        VisConstants.robotToCameraFront),
                    new VisionIOPhoton(
                        VisConstants.leftPho,
                        VisConstants.robotToCameraLeft),
                    new VisionIOPhoton(
                        VisConstants.rightPho,
                        VisConstants.robotToCameraRight));
            case SIM, REPLAY ->
                new Vision(
                    Drive.getInstance()::accept,
                    new VisionIO() {});
          };
    }

    return instance;
  }

  @AutoLogOutput(key = "Vision/Vision Enabled?")
  private boolean updateVision = true;

  @AutoLogOutput(key = "Vision/Enable Front?")
  private boolean frontEnable = true;

  @AutoLogOutput(key = "Vision/Using Multi Tag Only?")
  private boolean usingMultiTagOnly;

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final DigitalInput vrm;

  /*
   * Immediately enters multi-tag-only mode when any enabled camera sees
   * multiple tags. It waits 0.5 seconds after losing multi-tag observations
   * before allowing single-tag fallback again.
   */
  private final Debouncer multiTagDebouncer =
      new Debouncer(0.5, DebounceType.kFalling);

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    vrm = new DigitalInput(2);

    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      inputs[cameraIndex] = new VisionIOInputsAutoLogged();

      disconnectedAlerts[cameraIndex] =
          new Alert(
              "Vision camera " + cameraIndex + " is disconnected.",
              AlertType.kWarning);
    }
  }

  public void enableVision(boolean enable) {
    updateVision = enable;
  }

  public void enableFront(boolean enable) {
    frontEnable = enable;
  }

  /**
   * Returns the X angle to the latest target seen by a camera.
   *
   * @param cameraIndex index of the camera
   * @return target horizontal angle
   * @throws IndexOutOfBoundsException if the camera index is invalid
   */
  public Rotation2d getTargetX(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= inputs.length) {
      throw new IndexOutOfBoundsException(
          "Invalid vision camera index: " + cameraIndex);
    }

    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    updateInputsAndAlerts();

    Logger.recordOutput("Vision/VRM Is Good", vrm.get());

    if (!updateVision) {
      usingMultiTagOnly = false;
      Logger.recordOutput("Vision/Raw Multi Tag Available", false);
      clearSummaryLogs();
      return;
    }

    int firstEnabledCameraIndex = frontEnable ? 0 : 1;

    boolean rawMultiTagAvailable =
        hasMultiTagObservation(firstEnabledCameraIndex);

    /*
     * Rising transitions are immediate. Falling transitions are delayed by
     * the debouncer so a momentary loss of one tag does not immediately cause
     * the pose estimator to trust a single-tag measurement.
     */
    usingMultiTagOnly =
        multiTagDebouncer.calculate(rawMultiTagAvailable);

    Logger.recordOutput(
        "Vision/Raw Multi Tag Available",
        rawMultiTagAvailable);
    Logger.recordOutput(
        "Vision/Using Single Tag Fallback",
        !usingMultiTagOnly);

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    for (int cameraIndex = firstEnabledCameraIndex;
        cameraIndex < io.length;
        cameraIndex++) {
      processCamera(
          cameraIndex,
          allTagPoses,
          allRobotPoses,
          allRobotPosesAccepted,
          allRobotPosesRejected);
    }

    Logger.recordOutput(
        "Vision/Summary/TagPoses",
        allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses",
        allRobotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(Pose3d[]::new));
  }

  private void updateInputsAndAlerts() {
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].updateInputs(inputs[cameraIndex]);

      Logger.processInputs(
          "Vision/Camera" + cameraIndex,
          inputs[cameraIndex]);

      /*
       * Update every alert even when vision fusion or the front camera is
       * disabled. This prevents a disabled camera's alert from becoming stale.
       */
      disconnectedAlerts[cameraIndex].set(
          !inputs[cameraIndex].frontConnected);
    }
  }

  private boolean hasMultiTagObservation(int firstEnabledCameraIndex) {
    for (int cameraIndex = firstEnabledCameraIndex;
        cameraIndex < inputs.length;
        cameraIndex++) {
      for (var observation : inputs[cameraIndex].poseObservations) {
        if (observation.tagCount() > 1) {
          return true;
        }
      }
    }

    return false;
  }

  private void processCamera(
      int cameraIndex,
      List<Pose3d> allTagPoses,
      List<Pose3d> allRobotPoses,
      List<Pose3d> allRobotPosesAccepted,
      List<Pose3d> allRobotPosesRejected) {
    List<Pose3d> tagPoses = new LinkedList<>();
    List<Pose3d> robotPoses = new LinkedList<>();
    List<Pose3d> robotPosesAccepted = new LinkedList<>();
    List<Pose3d> robotPosesRejected = new LinkedList<>();

    for (int tagId : inputs[cameraIndex].tagIds) {
      aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
    }

    for (var observation : inputs[cameraIndex].poseObservations) {
      Pose3d estimatedPose = observation.pose();

      robotPoses.add(estimatedPose);

      boolean rejectPose =
          shouldRejectObservation(
              observation.tagCount(),
              observation.ambiguity(),
              observation.averageTagDistance(),
              observation.timestamp(),
              estimatedPose);

      if (rejectPose) {
        robotPosesRejected.add(estimatedPose);
        continue;
      }

      robotPosesAccepted.add(estimatedPose);

      double standardDeviationFactor =
          Math.pow(observation.averageTagDistance(), 2.0)
              / observation.tagCount();

      double linearStdDev =
          linearStdDevBaseline * standardDeviationFactor;
      double angularStdDev =
          angularStdDevBaseline * standardDeviationFactor;

      if (observation.type() == PoseObservationType.MEGATAG_2) {
        linearStdDev *= linearStdDevMegatag2Factor;
        angularStdDev *= angularStdDevMegatag2Factor;
      }

      if (cameraIndex < cameraStdDevFactors.length) {
        linearStdDev *= cameraStdDevFactors[cameraIndex];
        angularStdDev *= cameraStdDevFactors[cameraIndex];
      }

      if (!isValidStandardDeviation(linearStdDev)
          || !isValidStandardDeviation(angularStdDev)) {
        /*
         * Do not send malformed or overconfident covariance values to the pose
         * estimator. Move the observation from accepted to rejected logging.
         */
        robotPosesAccepted.remove(robotPosesAccepted.size() - 1);
        robotPosesRejected.add(estimatedPose);
        continue;
      }

      consumer.accept(
          estimatedPose.toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(
              linearStdDev,
              linearStdDev,
              angularStdDev));
    }

    String cameraLogKey = "Vision/Camera" + cameraIndex;

    Logger.recordOutput(
        cameraLogKey + "/TagPoses",
        tagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        cameraLogKey + "/RobotPoses",
        robotPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(
        cameraLogKey + "/RobotPosesAccepted",
        robotPosesAccepted.toArray(Pose3d[]::new));
    Logger.recordOutput(
        cameraLogKey + "/RobotPosesRejected",
        robotPosesRejected.toArray(Pose3d[]::new));

    allTagPoses.addAll(tagPoses);
    allRobotPoses.addAll(robotPoses);
    allRobotPosesAccepted.addAll(robotPosesAccepted);
    allRobotPosesRejected.addAll(robotPosesRejected);
  }

  private boolean shouldRejectObservation(
      int tagCount,
      double ambiguity,
      double averageTagDistance,
      double timestamp,
      Pose3d estimatedPose) {
    if (tagCount <= 0) {
      return true;
    }

    /*
     * When any enabled camera has a multi-tag estimate, reject every
     * single-tag estimate from every enabled camera.
     */
    if (usingMultiTagOnly && tagCount < 2) {
      return true;
    }

    /*
     * Single-tag observations are accepted only during fallback and only when
     * PhotonVision provides a valid, sufficiently low ambiguity value.
     */
    if (tagCount == 1
        && (!Double.isFinite(ambiguity)
            || ambiguity < 0.0
            || ambiguity > maxAmbiguity)) {
      return true;
    }

    if (!Double.isFinite(timestamp)
        || !Double.isFinite(averageTagDistance)
        || averageTagDistance <= 0.0) {
      return true;
    }

    return !isPoseValid(estimatedPose);
  }

  private boolean isPoseValid(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double z = pose.getZ();

    double roll = pose.getRotation().getX();
    double pitch = pose.getRotation().getY();
    double yaw = pose.getRotation().getZ();

    return Double.isFinite(x)
        && Double.isFinite(y)
        && Double.isFinite(z)
        && Double.isFinite(roll)
        && Double.isFinite(pitch)
        && Double.isFinite(yaw)
        && Math.abs(z) <= maxZError
        && x >= 0.0
        && x <= aprilTagLayout.getFieldLength()
        && y >= 0.0
        && y <= aprilTagLayout.getFieldWidth();
  }

  private boolean isValidStandardDeviation(double standardDeviation) {
    return Double.isFinite(standardDeviation)
        && standardDeviation > 0.0;
  }

  private void clearSummaryLogs() {
    Pose3d[] emptyPoses = new Pose3d[0];

    Logger.recordOutput("Vision/Summary/TagPoses", emptyPoses);
    Logger.recordOutput("Vision/Summary/RobotPoses", emptyPoses);
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        emptyPoses);
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        emptyPoses);
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}