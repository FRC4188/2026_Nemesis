package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ScoringCommands {
  public static InterpolatingDoubleTreeMap theLeftTree =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.8215051131055264, 2200.0),
          Map.entry(2.0, 2100.0),
          Map.entry(3.275369307926615, 2300.0),
          Map.entry(3.595707710420995, 2300.0),
          Map.entry(2.1367730463892607, 2200.0),
          Map.entry(1.7612802191261636, 2000.0),
          Map.entry(2.8377080004414252, 2200.0),
          Map.entry(4.480608710542746, 2400.0),
          Map.entry(3.5215872192835054, 2350.0),
          Map.entry(5.033825137547001, 2500.0),
          Map.entry(3.947989647341449, 2400.0),
          Map.entry(4.068028454299497, 2400.0),
          Map.entry(2.6433163458634867, 2200.0),
          Map.entry(5.289323328222003, 2575.0),
          Map.entry(4.979638580003689, 2575.0),
          Map.entry(5.098624335292876, 2575.0));

  public static InterpolatingDoubleTreeMap theRightTree =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0));

  public static LoggedNetworkNumber _RPML = new LoggedNetworkNumber("Aim Tuning/Left RPM", 0.0);
  public static LoggedNetworkNumber _RPMR = new LoggedNetworkNumber("Aim Tuning/Right RPM", 0.0);

  public static Command dataShoot(Shooter shooter, Hopper hopper) {
    return Commands.parallel(
        Commands.runEnd(
            () -> shooter.setVelocityRPM(_RPML.getAsDouble(), _RPMR.getAsDouble()),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                Commands.repeatingSequence(
                    new WaitUntilCommand(() -> shooter.atGoal()),
                    Commands.runOnce(() -> hopper.runHopperVolts(6.0, 6.0), hopper),
                    new WaitUntilCommand(() -> !shooter.atGoal()),
                    Commands.runOnce(() -> hopper.runHopperVolts(0.0, 0.0), hopper)))
            .finallyDo(hopper::stop));
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
    // new WaitCommand(0.1)
    //     .andThen(
    //         Commands.repeatingSequence(
    //             new WaitUntilCommand(() -> shooter.atGoal()),
    //             Commands.runOnce(() -> hopper.runHopperVolts(6.0, 6.0), hopper),
    //             new WaitUntilCommand(() -> !shooter.atGoal()),
    //             Commands.runOnce(() -> hopper.runHopperVolts(0.0, 0.0), hopper)))
    //     .finallyDo(hopper::stop));
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
    //return 145.557 * distance + 1806.67131;
    return theLeftTree.get(distance);
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

    return Commands.sequence(
        Commands.run(() -> wrist.runWristVolts(-6), wrist).withTimeout(0.12),
        Commands.run(() -> wrist.runWristVolts(8), wrist).withTimeout(0.12),
        Commands.run(() -> wrist.runWristVolts(-8), wrist)
            .until(() -> wrist.getAngle() < 30)
            .finallyDo(wrist::stop));
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
