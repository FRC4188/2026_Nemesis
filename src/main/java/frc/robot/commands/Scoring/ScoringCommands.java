package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ScoringCommands {
  private static final Shooter shooter = Shooter.getInstance();
  private static final Hopper hopper = Hopper.getInstance();
  private static final Drive drive = Drive.getInstance();
  private static final Hood hood = Hood.getInstance();
  private static final Wrist wrist = Wrist.getInstance();

  public static LoggedNetworkNumber _RPM = new LoggedNetworkNumber("Aim Tuning/RPM", 0.0);

  public static Command dataShoot() {
    return Commands.parallel(
        Commands.runEnd(
            () -> shooter.setVelocityRPM(_RPM.getAsDouble(), _RPM.getAsDouble()),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                Commands.repeatingSequence(
                    new WaitUntilCommand(() -> shooter.atGoal()),
                    Commands.runOnce(() -> hopper.runHopperVolts(6.0, 4.0), hopper),
                    new WaitUntilCommand(() -> !shooter.atGoal()),
                    Commands.runOnce(() -> hopper.runHopperVolts(0.0, 0.0), hopper)))
            .finallyDo(hopper::stop));
  }

  public static Command staticAim() {
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

  public static Command staticShoot() {
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
                            () -> hopper.runHopperVolts(6.0, 4.0), hopper::stop, hopper))));
  }

  public static Command manualAim(DoubleSupplier distance) {
    return Commands.runEnd(
        () -> hood.setAngle(inclineHueristic(Units.feetToMeters(distance.getAsDouble()))),
        hood::stow,
        hood);
  }

  public static Command manualShoot(DoubleSupplier distance) {
    return Commands.parallel(
        Commands.runEnd(
            () ->
                shooter.setVelocityRPM(
                    RPMRegress(Units.feetToMeters(distance.getAsDouble())),
                    RPMRegress(Units.feetToMeters(distance.getAsDouble()))),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(6.0, 4.0), hopper::stop, hopper))));
  }

  public static double RPMRegress(double distance) {
    return 145.557 * distance + 1806.67131;
  }

  public static Rotation2d inclineHueristic(double distance) {
    return Rotation2d.fromRadians(Math.PI / 2 - Math.atan(7.0 / distance));
  }

  public static Command passAim() {
    return Commands.runEnd(() -> hood.setAngle(Rotation2d.fromDegrees(40)), hood::stow, hood);
  }

  public static Command passShoot() {
    return Commands.parallel(
        Commands.runEnd(() -> shooter.setVelocityRPM(3100, 3100), shooter::stop, shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(6.0, 4.0), hopper::stop, hopper))));
  }

  public static Command shake() {
    return Commands.either(
        Commands.repeatingSequence(
                Commands.run(() -> wrist.runWristVolts(5), wrist).withTimeout(0.2),
                Commands.run(() -> wrist.runWristVolts(-5), wrist).withTimeout(0.2))
            .until(() -> wrist.getAngle() > 120.0)
            .finallyDo(wrist::stop),
        Commands.none(),
        () -> wrist.shakeEnable);
  }

  public static Command downNoStall() {
    return Commands.run(() -> wrist.runWristVolts(-4), wrist)
        .until(() -> wrist.getAngle() < 30)
        .finallyDo(wrist::stop);
  }

  public static Command forceDown() {

    return Commands.sequence(
        Commands.run(() -> wrist.runWristVolts(-6), wrist).withTimeout(0.12),
        Commands.run(() -> wrist.runWristVolts(8), wrist).withTimeout(0.12),
        Commands.run(() -> wrist.runWristVolts(-8), wrist)
            .until(() -> wrist.getAngle() < 30)
            .finallyDo(wrist::stop));
  }

  public static Command goodStow() {
    return Commands.run(() -> wrist.runWristVolts(6), wrist)
        .until(() -> wrist.getAngle() > 120)
        .finallyDo(wrist::stop);
  }
}
