package frc.robot.CSPLib.csppathing;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

/** Path building helper that uses CSPPilot. */
public class CSPPathing {
  private static final Drive drive = Drive.getInstance();

  private final CSPPilot pilot;
  private final ProfiledPIDController headingController;
  private final List<PathPlannerPath> paths = new ArrayList<>();

  private Pose2d startPose = null;

  public CSPPathing(CSPPilot pilot, ProfiledPIDController headingController) {
    this.pilot = pilot;
    this.headingController = headingController;
  }

  public CSPPathing withStartPose(Pose2d pose) {
    this.startPose = pose;
    return this;
  }

  public CSPPathing addPath(PathPlannerPath path) {
    paths.add(path);
    return this;
  }

  public CSPPathing addPaths(PathPlannerPath... morePaths) {
    for (PathPlannerPath path : morePaths) {
      addPath(path);
    }
    return this;
  }

  public Command build() {
    if (paths.isEmpty()) {
      throw new IllegalStateException("No paths defined");
    }

    final int lastIndex = paths.size() - 1;
    final int[] index = new int[] {0};
    final CSPPilot.PathFollower[] followers = new CSPPilot.PathFollower[paths.size()];

    return Commands.run(
            () -> {
              Pose2d pose = drive.getPose();
              ChassisSpeeds robotSpeeds = drive.getChassisSpeeds();

              while (index[0] < lastIndex
                  && followers[index[0]] != null
                  && followers[index[0]].isFinished(pose)) {
                index[0]++;
              }

              CSPPilot.PathFollower follower = followers[index[0]];
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
        .beforeStarting(
            () -> {
              index[0] = 0;

              Pose2d reset = startPose != null ? startPose : drive.getPose();
              if (startPose != null) {
                drive.setPose(startPose);
              }
              headingController.reset(reset.getRotation().getRadians());

              for (int i = 0; i < paths.size(); i++) {
                followers[i] = pilot.followPath(paths.get(i));
                followers[i].reset();
              }
            })
        .until(
            () -> followers[lastIndex] != null && followers[lastIndex].isFinished(drive.getPose()))
        .finallyDo(interrupted -> drive.stop());
  }
}
