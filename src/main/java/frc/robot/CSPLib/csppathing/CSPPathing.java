package frc.robot.CSPLib.csppathing;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class CSPPathing {
  private static final Drive drive = Drive.getInstance();

  private final CSPPilot pilot;
  private final ProfiledPIDController headingController;
  private final List<PathPlannerPath> paths = new ArrayList<>();
  private Pose2d startPose = null;
  private RobotConfig robotConfig = null;

  public CSPPathing(CSPPilot pilot, ProfiledPIDController headingController) {
    this.pilot = Objects.requireNonNull(pilot, "pilot cannot be null");
    this.headingController =
        Objects.requireNonNull(headingController, "headingController cannot be null");
  }

  public CSPPathing withStartPose(Pose2d pose) {
    this.startPose = pose;
    return this;
  }

  public CSPPathing withRobotConfig(RobotConfig robotConfig) {
    this.robotConfig = robotConfig;
    return this;
  }

  public CSPPathing addPath(PathPlannerPath path) {
    paths.add(Objects.requireNonNull(path, "path cannot be null"));
    return this;
  }

  public CSPPathing addPaths(PathPlannerPath... morePaths) {
    Objects.requireNonNull(morePaths, "morePaths cannot be null");
    for (PathPlannerPath path : morePaths) {
      addPath(path);
    }
    return this;
  }

  public Command build() {
    if (paths.isEmpty()) {
      throw new IllegalStateException("No paths defined");
    }

    List<Command> segments = new ArrayList<>();

    for (int i = 0; i < paths.size(); i++) {
      final PathPlannerPath path = paths.get(i);
      final boolean firstSegment = i == 0;

      final CSPPilot.PathSeed seed =
          robotConfig != null ? pilot.seedFrom(path, robotConfig) : pilot.seedFrom(path);

      final CSPPilot.PathFollower follower = pilot.followPath(path, seed.constraints(), 1000, 1000);

      Command segment =
          Commands.sequence(
              Commands.runOnce(
                  () -> {
                    follower.reset();
                    if (firstSegment && startPose != null) {
                      drive.setPose(startPose);
                    }
                    Pose2d pose = drive.getPose();
                    headingController.reset(pose.getRotation().getRadians());
                  },
                  drive),
              Commands.run(
                      () -> {
                        Pose2d pose = drive.getPose();
                        ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();
                        CSPPilot.CSPResult out = follower.update(pose, robotSpeeds);

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
                  .until(() -> follower.isFinished(drive.getPose())));

      segments.add(segment);
    }

    return Commands.sequence(segments.toArray(new Command[0]))
        .beforeStarting(
            () -> {
              if (startPose != null) {
                drive.setPose(startPose);
              }
              headingController.reset(drive.getPose().getRotation().getRadians());
            })
        .finallyDo(
            interrupted -> {
              drive.stop();
            });
  }

  public static Command buildTrialAuto(CSPPilot pilot, ProfiledPIDController headingController) {
    CSPPathing path =
        new CSPPathing(pilot, headingController)
            .withStartPose(
                new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
            .addPath(
                PathBuilder.build(
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_center.plus(
                                    new Translation2d(0, -0.18)),
                                Rotation2d.kCCW_90deg))
                        .withStartingSpeed(5)
                        .withStartingRotation(Rotation2d.kCCW_90deg)
                        .withOverrideRotations(
                            new RotationTarget(0.97, Rotation2d.fromDegrees(87.075)),
                            new RotationTarget(0.60, Rotation2d.kCCW_90deg),
                            new RotationTarget(2.00, Rotation2d.fromDegrees(110.726)),
                            new RotationTarget(3.00, Rotation2d.fromDegrees(-95.856)),
                            new RotationTarget(3.34, Rotation2d.fromDegrees(-85.402)))
                        .withHeading(Rotation2d.fromDegrees(61.763))
                        .withControlDistances(0, 0.250),
                    new PathBuilder.Target(new Pose2d(7.355, 1.523 - 0.18, Rotation2d.kZero))
                        .withHeading(Rotation2d.fromDegrees(66.360))
                        .withControlDistances(1.517, 0.476),
                    new PathBuilder.Target(new Pose2d(7.614, 3.051, Rotation2d.kZero))
                        .withHeading(Rotation2d.fromDegrees(120.689))
                        .withControlDistances(0.288, 1.250),
                    new PathBuilder.Target(new Pose2d(5.968, 3.051, Rotation2d.kZero))
                        .withHeading(Rotation2d.fromDegrees(-104.349))
                        .withControlDistances(0.955, 0.310),
                    new PathBuilder.Target(new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
                        .withHeading(Rotation2d.fromDegrees(99.792))
                        .withControlDistances(0.250, 0)
                        .withEndingRotation(Rotation2d.kZero)
                        .withEndingSpeed(2)));

    return path.build();
  }
}
