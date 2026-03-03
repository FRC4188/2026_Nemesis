package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO;

public class DriveToCluster extends Command {

  Pose2d target;
  DriveToPose driveToPose;
  VisionIO vision;
  Drive drive;

  public DriveToCluster(Drive drive, VisionIO vision, Pose3d drivePose) {
    this.vision = vision;
    this.drive = drive;
    target = vision.getCluster(drivePose);
    driveToPose = new DriveToPose(drive, () -> target);
  }

  @Override
  public void initialize() {
    driveToPose.initialize();
  }

  @Override
  public void execute() {
    driveToPose.execute();
  }

  @Override
  public void end(boolean interrupted) {
    driveToPose.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return driveToPose.isFinished();
  }

  public Translation2d PoseToTranslation(Pose2d pose) {
    return pose.getTranslation();
  }

  public Pose2d TranslationToPose(Translation2d translation) {
    return new Pose2d(translation, translation.getAngle());
  }
}
