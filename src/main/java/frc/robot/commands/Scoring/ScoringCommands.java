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
import frc.robot.subsystems.intake.Intake;
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
  private static final Intake intake = Intake.getInstance();

  public static LoggedNetworkNumber _RPM = new LoggedNetworkNumber("Aim Tuning/RPM", 0.0);

  public static Command dataShoot() {
    return Commands.parallel(
        Commands.runEnd(
            () -> shooter.setVelocityRPM(_RPM.getAsDouble(), _RPM.getAsDouble()),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(8.0, 4.0), hopper::stop, hopper))));
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
                            () -> hopper.runHopperVolts(8.0, 4.0), hopper::stop, hopper))));
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
                            () -> hopper.runHopperVolts(8.0, 4.0), hopper::stop, hopper))));
  }

  public static double RPMRegress(double distance) {
    // return 145.557 * distance + 1806.67131;
    return (11.94806 * Math.pow(distance, 3))
        - (92.62501 * Math.pow(distance, 2))
        + 351.50335 * distance
        + 1736.74591;
  }

  public static Rotation2d inclineHueristic(double distance) {
    // return Rotation2d.fromRadians(Math.PI / 2 - Math.atan(7 / distance));
    return Rotation2d.fromRadians(Math.PI / 2 - Math.atan(8.5 / distance));
  }

  public static Command passAim() {
    return Commands.runEnd(() -> hood.setAngle(Rotation2d.fromDegrees(40)), hood::stow, hood);
  }

  public static Command passShoot() {
    return Commands.parallel(
        Commands.runEnd(() -> shooter.setVelocityRPM(3000, 3000), shooter::stop, shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(
                            () -> hopper.runHopperVolts(8.0, 4.0), hopper::stop, hopper))));
  }

  public static Command shake() {
    return Commands.either(
        Commands.parallel(
            Commands.repeatingSequence(
                    Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.25),
                    Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.25))
                .until(() -> wrist.getAngle() > 120.0)
                .finallyDo(wrist::stop),
            Commands.runEnd(() -> intake.intakeVolts(2.0), intake::stop, intake)),
        Commands.none(),
        () -> wrist.shakeEnable);
  }

  public static Command fullShake() {
    // return Commands.runEnd(() -> wrist.runWristVolts(3.5), () -> wrist.stop(), wrist)
    //     .withTimeout(3)
    //     .until(() -> wrist.getAngle() > 120.0);

    // ADD WRIST REQUIREMENTS (if we even use this one)
    // return Commands.either(
    //     Commands.sequence(
    //         new WaitCommand(1.25),
    //         Commands.run(() -> wrist.runWristVolts(3)).withTimeout(0.5),
    //         Commands.run(() -> wrist.runWristVolts(-3)).withTimeout(0.5),
    //         new WaitCommand(0.75),
    //         Commands.run(() -> wrist.runWristVolts(3)).withTimeout(0.5),
    //         Commands.run(() -> wrist.runWristVolts(-3)).withTimeout(0.5),
    //         new WaitCommand(0.75),
    //         Commands.run(() -> wrist.runWristVolts(3)).withTimeout(0.5),
    //         Commands.run(() -> wrist.runWristVolts(-3)).withTimeout(0.5),
    //         new WaitCommand(0.75),
    //         Commands.run(() -> wrist.runWristVolts(3)).withTimeout(0.5),
    //         Commands.run(() -> wrist.runWristVolts(-3)).withTimeout(0.5),
    //         new WaitCommand(0.1),
    //         Commands.run(() -> wrist.runWristVolts(3))
    //             .withTimeout(2.5)
    //             .until(() -> wrist.getAngle() > 110.0))
    //     .until(() -> wrist.getAngle() > 110.0), Commands.none(), () -> wrist.shakeEnable);

    // return Commands.either(
    //     Commands.sequence(
    //             new WaitCommand(1),
    //             Commands.run(() -> wrist.runWristVolts(3), wrist)
    //                 .withTimeout(0.5)
    //                 .until(() -> wrist.isStalled()),
    //             Commands.run(() -> wrist.runWristVolts(-3), wrist)
    //                 .until(() -> wrist.isStalled()),
    //             new WaitCommand(0.1),
    //             Commands.run(() -> wrist.runWristVolts(3), wrist)
    //                 .withTimeout(0.5)
    //                 .until(() -> wrist.isStalled()),
    //             Commands.run(() -> wrist.runWristVolts(-3), wrist)
    //                 .until(() -> wrist.isStalled()),
    //             new WaitCommand(0.1),
    //             Commands.run(() -> wrist.runWristVolts(3), wrist)
    //                 .withTimeout(0.5)
    //                 .until(() -> wrist.isStalled()),
    //             Commands.run(() -> wrist.runWristVolts(-3), wrist)
    //                 .until(() -> wrist.isStalled()),
    //             new WaitCommand(0.1),
    //             Commands.run(() -> wrist.runWristVolts(3), wrist)
    //                 .withTimeout(0.5)
    //                 .until(() -> wrist.isStalled()),
    //             Commands.run(() -> wrist.runWristVolts(-3), wrist)
    //                 .until(() -> wrist.isStalled()),
    //             new WaitCommand(0.1),
    //             Commands.parallel(
    //                 Commands.sequence(
    //                     Commands.run(() -> wrist.runWristVolts(3), wrist)
    //                         .withTimeout(0.6)
    //                         .until(() -> wrist.isStalled()),
    //                         new WaitCommand(0.1),
    //                     Commands.run(() -> wrist.runWristVolts(-3), wrist)
    //                         .until(() -> wrist.isStalled())),
    //                 Commands.runEnd(() -> intake.intakeVolts(2.0), intake::stop, intake)
    //                     .withTimeout(1)),
    //             new WaitCommand(0.1),
    //             Commands.parallel(
    //                 Commands.run(() -> wrist.runWristVolts(3), wrist)
    //                     .until(() -> wrist.getAngle() > 80.0 || wrist.isStalled()),
    //                 Commands.runEnd(() -> intake.intakeVolts(2.0), intake::stop, intake)
    //                     .withTimeout(2.5)))
    //         .until(() -> wrist.getAngle() > 80.0),
    //     Commands.none(),
    //     () -> wrist.shakeEnable);

    return Commands.either(
        Commands.sequence(
                new WaitCommand(1),
                Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.5),
                Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.5),
                new WaitCommand(0.1),
                Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.5),
                Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.5),
                new WaitCommand(0.1),
                Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.5),
                Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.5),
                new WaitCommand(0.1),
                Commands.parallel(
                    Commands.sequence(
                        Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.5),
                        new WaitCommand(0.1),
                        Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.5),
                        Commands.runEnd(() -> intake.intakeVolts(2.0), intake::stop, intake)
                            .withTimeout(0.1))),
                new WaitCommand(0.1),
                Commands.parallel(
                    Commands.run(() -> wrist.runWristVolts(3), wrist)
                        .until(() -> wrist.getAngle() > 80.0),
                    Commands.runEnd(() -> intake.intakeVolts(2.0), intake::stop, intake)
                        .withTimeout(2.5)))
            // .until(() -> wrist.getAngle() > 80.0)
            .finallyDo(() -> wrist.stop()),
        Commands.none(),
        () -> wrist.shakeEnable);

    // test; pls work
    // return Commands.either(
    //     Commands.repeatingSequence(
    //             Commands.run(() -> wrist.runWristVolts(3),
    // wrist).withTimeout(help.getAsDouble()),
    //             Commands.run(() -> wrist.runWristVolts(-3), wrist)
    //                 .until(() -> wrist.getAngle() < 10))
    //         .until(() -> wrist.getAngle() > 120.0)
    //         .finallyDo(() -> wrist.stop()),
    //     Commands.none(),
    //     () -> wrist.shakeEnable);
  }

  public static Command halfShake() {
    return Commands.either(
        Commands.sequence(
                new WaitCommand(0.5),
                Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.5),
                Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.5),
                new WaitCommand(0.25),
                Commands.run(() -> wrist.runWristVolts(3), wrist).withTimeout(0.5),
                Commands.run(() -> wrist.runWristVolts(-3), wrist).withTimeout(0.5),
                new WaitCommand(0.30),
                Commands.run(() -> wrist.runWristVolts(3), wrist)
                    .until(() -> wrist.getAngle() > 80.0))
            .finallyDo(() -> wrist.stop()),
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
