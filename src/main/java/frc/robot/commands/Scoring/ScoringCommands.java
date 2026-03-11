package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ScoringCommands {
  public static LoggedNetworkNumber _RPM = new LoggedNetworkNumber("Aim Tuning/RPM", 0.0);

  public static Command dataShoot(Shooter shooter, Hopper hopper) {
    return Commands.parallel(
        Commands.runEnd(() -> shooter.setVelocityRPM(_RPM.getAsDouble()), shooter::stop, shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(() -> hopper.runHopperVolts(8.5), hopper::stop, hopper))));
  }

  public static Command staticAim(Drive drive, Hood hood) {
    return Commands.runEnd(
        () ->
            hood.setAngle(
                inclineHueristic(
                    AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getNorm())),
        hood::stow,
        hood);
  }

  public static Command staticShoot(Drive drive, Shooter shooter, Hopper hopper) {
    return Commands.parallel(
        Commands.runEnd(
            () ->
                shooter.setVelocityRPM(
                    RPMRegress(
                        AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                            .minus(drive.getPose().getTranslation())
                            .getNorm())),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(() -> hopper.runHopperVolts(8.5), hopper::stop, hopper))));
  }

  public static double RPMRegress(double distance) {
    return 233.2242 * distance + 1523.17052;
  }

  public static Rotation2d inclineHueristic(double distance) {
    return Rotation2d.fromRadians(Math.PI / 2 - Math.atan(7 / distance));
  }

  public static Command passAim(Hood hood) {
    return Commands.runEnd(() -> hood.setAngle(Rotation2d.fromDegrees(60)), hood::stow, hood);
  }

  public static Command passShoot(Shooter shooter) {
    return Commands.runEnd(() -> shooter.setVelocityRPM(2500), shooter::stop, shooter);
  }

  public static Command shake(Wrist wrist) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> wrist.runWristVolts(1.5)),
            new WaitUntilCommand(() -> wrist.isStalled()),
            Commands.runOnce(wrist::stop),
            new WaitCommand(0.5))
        .until(() -> wrist.getAngle() > 120.0)
        .finallyDo(() -> wrist.stow());
  }

  public static Command downWoStall(Wrist wrist) {
    return Commands.sequence(
            Commands.runOnce(wrist::down, wrist),
            new WaitUntilCommand(() -> wrist.getAngle() < 30.0 || wrist.isStalled()))
        .finallyDo(wrist::stop);
  }

  // TODO: add poses
  public static Command goToClimb(Drive drive, Climber climb) {
    return Commands.sequence(
        Commands.runOnce(climb::lower),
        new DriveToPose(
            drive,
            () ->
                drive
                    .getPose()
                    .nearest(
                        List.of(
                            AllianceFlip.apply(Pose2d.kZero), AllianceFlip.apply(Pose2d.kZero)))),
        Commands.runOnce(climb::raise),
        new DriveToPose(
            drive,
            () ->
                drive
                    .getPose()
                    .nearest(
                        List.of(
                            AllianceFlip.apply(Pose2d.kZero), AllianceFlip.apply(Pose2d.kZero)))));
  }
}
