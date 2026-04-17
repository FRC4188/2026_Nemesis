package frc.robot.CSPLib.csppathing;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.*;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;

public final class PosePathing {

  private static final Drive drive = Drive.getInstance();

  private static final APConstraints baseConstraints =
      new APConstraints()
          .withAcceleration(Constants.DriveConstants.DRIVE_MAXACC)
          .withJerk(2.0)
          .withVelocity(Constants.DriveConstants.DRIVE_MAXVEL * 0.8);

  private static final APProfile baseProfile =
      new APProfile(baseConstraints)
          .withErrorXY(Centimeters.of(2))
          .withErrorTheta(Degrees.of(0.5))
          .withBeelineRadius(Centimeters.of(5));

  private static final Autopilot baseAutopilot = new Autopilot(baseProfile);
  private final ProfiledPIDController headingController;
  private final List<Waypoint> waypoints = new ArrayList<>();

  private double handoffThresholdMeters = 0.25;
  private Pose2d startPose = null;

  // =========================
  // WAYPOINT (NOW HAS SPEED)
  // =========================
  private static final class Waypoint {
    final Pose2d pose;
    final APTarget target;
    final APConstraints constraints; // NEW

    Waypoint(Pose2d pose, APTarget target, APConstraints constraints) {
      this.pose = pose;
      this.target = target;
      this.constraints = constraints;
    }
  }

  public PosePathing(ProfiledPIDController headingController) {
    this.headingController = Objects.requireNonNull(headingController);
  }

  public PosePathing withStartPose(Pose2d pose) {
    this.startPose = pose;
    return this;
  }

  public PosePathing withHandoffThreshold(double meters) {
    this.handoffThresholdMeters = meters;
    return this;
  }

  // =========================
  // WAYPOINT ADD (NO SPEED)
  // =========================
  public PosePathing addWaypoint(Pose2d pose) {
    return addWaypoint(pose, t -> t, null);
  }

  // =========================
  // WAYPOINT ADD (WITH CONFIG)
  // =========================
  public PosePathing addWaypoint(Pose2d pose, Function<APTarget, APTarget> config) {
    return addWaypoint(pose, config, null);
  }

  // =========================
  // WAYPOINT ADD (WITH SPEED)
  // =========================
  public PosePathing addWaypoint(
      Pose2d pose, Function<APTarget, APTarget> config, APConstraints speedConstraints) {

    APTarget target = config.apply(new APTarget(pose));

    waypoints.add(new Waypoint(pose, target, speedConstraints));
    return this;
  }

  // convenience overload
  public PosePathing addWaypoint(Pose2d pose, APTarget target, APConstraints speedConstraints) {

    waypoints.add(new Waypoint(pose, target, speedConstraints));
    return this;
  }

  public Command build() {

    if (waypoints.isEmpty()) {
      throw new IllegalStateException("No waypoints defined");
    }

    final int lastIndex = waypoints.size() - 1;
    final AtomicInteger index = new AtomicInteger(0);

    Command main =
        Commands.run(
                () -> {
                  Pose2d pose = drive.getPose();
                  ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();

                  // ===== Handoff logic =====
                  while (index.get() < lastIndex
                      && distance(pose, waypoints.get(index.get()).pose)
                          <= handoffThresholdMeters) {

                    index.incrementAndGet();
                    // headingController.reset(pose.getRotation().getRadians());
                  }

                  Waypoint wp = waypoints.get(index.get());

                  // =========================
                  // PER-WAYPOINT AUTOPILOT
                  // =========================
                  Autopilot autopilot =
                      (wp.constraints != null)
                          ? new Autopilot(
                              new APProfile(wp.constraints)
                                  .withErrorXY(Centimeters.of(2))
                                  .withErrorTheta(Degrees.of(0.5))
                                  .withBeelineRadius(Centimeters.of(20)))
                          : baseAutopilot;

                  APResult out = autopilot.calculate(pose, robotSpeeds, wp.target);

                  double omega =
                      headingController.calculate(
                          pose.getRotation().getRadians(), out.targetAngle().getRadians());

                  ChassisSpeeds speeds =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          out.vx().in(MetersPerSecond),
                          out.vy().in(MetersPerSecond),
                          omega,
                          pose.getRotation());

                  drive.runVelocity(speeds);
                },
                drive)
            .beforeStarting(
                () -> {
                  index.set(0);
                  Pose2d reset = startPose != null ? startPose : drive.getPose();
                  headingController.reset(reset.getRotation().getRadians());
                  if (startPose != null) drive.setPose(startPose);
                })
            .until(
                () ->
                    index.get() == lastIndex
                        && baseAutopilot.atTarget(drive.getPose(), waypoints.get(lastIndex).target))
            .finallyDo(interrupted -> drive.stop());

    return main;
  }

  private static double distance(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  // =========================
  // TRIAL AUTO EXAMPLE
  // =========================
  public static Command buildTrialAuto() {

    PosePathing path =
        new PosePathing(Constants.DriveConstants.ANGLE_PID)
            .withStartPose(
                new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
            .withHandoffThreshold(0.5)

            // fast segment
            .addWaypoint(
                new Pose2d(FieldConstants.FuelField.right_midline_corner, Rotation2d.kCCW_90deg),
                t -> t.withEntryAngle(Rotation2d.fromDegrees(45)).withVelocity(1),
                new APConstraints().withAcceleration(8.0).withJerk(3.0))

            // slow precise segment
            .addWaypoint(
                new Pose2d(
                    FieldConstants.field_center.plus(new Translation2d(-0.3, 0)),
                    Rotation2d.fromDegrees(110)),
                t -> t.withVelocity(1).withEntryAngle(Rotation2d.kCCW_90deg),
                new APConstraints().withAcceleration(3.0).withJerk(2.0).withVelocity(1));
    PosePathing path2 =
        new PosePathing(Constants.DriveConstants.ANGLE_PID)
            .withStartPose(
                new Pose2d(
                    FieldConstants.field_center.plus(new Translation2d(-0.3, 0)),
                    Rotation2d.fromDegrees(110)))
            .withHandoffThreshold(0.5)
            .addWaypoint(
                new Pose2d(FieldConstants.FuelField.right_close_corner, Rotation2d.kCCW_90deg),
                t ->
                    t.withVelocity(DriveConstants.DRIVE_MAXVEL)
                        .withEntryAngle(Rotation2d.kCW_90deg.plus(Rotation2d.fromDegrees(-20))))
            .addWaypoint(
                new Pose2d(
                    FieldConstants.Trench.right_trench_center.plus(new Translation2d(0, -0.3)),
                    Rotation2d.kCCW_90deg),
                t -> t.withVelocity(DriveConstants.DRIVE_MAXVEL).withEntryAngle(Rotation2d.k180deg))
            .addWaypoint(
                new Pose2d(
                    FieldConstants.Trench.right_trench_center.plus(new Translation2d(0, -0.3)),
                    Rotation2d.kCCW_90deg),
                t -> t.withVelocity(1).withoutEntryAngle())
            .addWaypoint(
                new Pose2d(
                    FieldConstants.Trench.right_trench_center.plus(new Translation2d(-0.40, -0.19)),
                    Rotation2d.kCCW_90deg),
                t -> t.withVelocity(0).withEntryAngle(Rotation2d.fromDegrees(-185)),
                new APConstraints()
                    .withAcceleration(Constants.DriveConstants.DRIVE_MAXACC)
                    .withJerk(2.0)
                    .withVelocity(1));

    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))),
        path.build(),
        path2.build());
  }
}
