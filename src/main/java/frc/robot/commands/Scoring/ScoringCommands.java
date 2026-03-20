package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

public class ScoringCommands {
  // public static LoggedNetworkNumber _RPM = new LoggedNetworkNumber("Aim Tuning/RPM", 0.0);

  //   public static Command dataShoot(Shooter shooter, Hopper hopper) {
  //     return Commands.parallel(
  //         // starting with right shooter first
  //         Commands.runEnd(
  //             () -> shooter.setVelocityRPM(_RPM.getAsDouble(), _RPM.getAsDouble()),
  //             shooter::stop,
  //             shooter),
  //         new WaitCommand(0.1)
  //             .andThen(
  //                 new WaitUntilCommand(() -> shooter.rightAtGoal())
  //                     .andThen(
  //                         Commands.runEnd(
  //                             () -> hopper.runHopperVolts(6.0, 6.0), hopper::stop, hopper))));
  //   }

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
                            .getNorm()),
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
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(6.0, 6.0), hopper::stop, hopper))));
  }

  public static Command manualAim(Hood hood) {
    return Commands.runEnd(
        () -> hood.setAngle(inclineHueristic(Units.feetToMeters(12.0))), hood::stow, hood);
  }

  public static Command manualShoot(Shooter shooter, Hopper hopper) {
    return Commands.parallel(
        Commands.runEnd(
            () ->
                shooter.setVelocityRPM(
                    RPMRegress(Units.feetToMeters(12.0)), RPMRegress(Units.feetToMeters(12.0))),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(6.0, 6.0), hopper::stop, hopper))));
  }

  public static double RPMRegress(double distance) {
    return 144.557 * distance + 1806.67131;
  }

  public static Rotation2d inclineHueristic(double distance) {
    return Rotation2d.fromRadians(Math.PI / 2 - Math.atan(7 / distance));
  }

  public static Command passAim(Hood hood) {
    return Commands.runEnd(() -> hood.setAngle(Rotation2d.fromDegrees(40)), hood::stow, hood);
  }

  public static Command passShoot(Shooter shooter, Hopper hopper) {
    return Commands.parallel(
        Commands.runEnd(() -> shooter.setVelocityRPM(3100, 3100), shooter::stop, shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(6.0, 6.0), hopper::stop, hopper))));
  }

  public static Command shake(Wrist wrist) {
    return Commands.either(
        Commands.repeatingSequence(
                Commands.run(() -> wrist.runWristVolts(5), wrist).withTimeout(0.25),
                Commands.run(() -> wrist.runWristVolts(-5), wrist).withTimeout(0.25))
            .until(() -> wrist.getAngle() > 120.0)
            .finallyDo(wrist::stop),
        Commands.none(),
        () -> wrist.shakeEnable);
  }

  public static Command downNoStall(Wrist wrist) {
    return Commands.run(() -> wrist.runWristVolts(-4), wrist)
        .until(() -> wrist.getAngle() < 30)
        .finallyDo(wrist::stop);
  }

  public static Command forceDown(Wrist wrist) {
    return Commands.run(() -> wrist.runWristVolts(-8), wrist)
        .until(() -> wrist.getAngle() < 30)
        .finallyDo(wrist::stop);
  }

  public static Command goodStow(Wrist wrist) {
    return Commands.run(() -> wrist.runWristVolts(6), wrist)
        .until(() -> wrist.getAngle() > 120)
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
