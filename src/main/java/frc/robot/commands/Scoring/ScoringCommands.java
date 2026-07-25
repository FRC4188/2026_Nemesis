package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
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
        Commands.runEnd(() -> shooter.setVelocityRPM(_RPM.getAsDouble()), shooter::stop, shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(() -> hopper.runHopper(9.0, 5000), hopper::stop, hopper))));
  }

  public static Command wristCompress() {
    return Commands.parallel(
        Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake),
        Commands.sequence(
                Commands.runEnd(() -> wrist.runWristVolts(5), wrist::stop, wrist)
                    .until(
                        () ->
                            wrist.getStatorCurrent() > Constants.WristConstants.fuelStatorCurrent),
                Commands.waitSeconds(0.12))
            .repeatedly()
            .until(() -> wrist.getAngle() > 90));
  }

  public static boolean initialShots = true;

  public static Command shoot(DoubleSupplier distance) {
    return Commands.either(
        Commands.either(
            Commands.parallel(
                Commands.runEnd(
                    () ->
                        hood.setAngle(
                            inclineHueristic(
                                AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                    .minus(drive.getPose().getTranslation())
                                    .getNorm())),
                    hood::stop,
                    hood),
                Commands.runEnd(
                    () ->
                        shooter.setVelocityRPM(
                            RPMRegress(
                                    AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                        .minus(drive.getPose().getTranslation())
                                        .getNorm())
                                + ((initialShots) ? 200 : 0)),
                    shooter::stop,
                    shooter),
                new WaitCommand(0.1)
                    .andThen(
                        new WaitUntilCommand(
                                () ->
                                    shooter.atGoal()
                                        && hood.atGoal()
                                        && drive.getRotation().getDegrees()
                                                - AllianceFlip.apply(
                                                        FieldConstants.Hub.hub_center_2d)
                                                    .minus(drive.getPose().getTranslation())
                                                    .getAngle()
                                                    .getDegrees()
                                            < 5)
                            .andThen(
                                Commands.parallel(
                                    Commands.runEnd(
                                        () -> hopper.runHopper(9.0, 5000), hopper::stop, hopper),
                                    new WaitCommand(0.1)
                                        .andThen(
                                            new WaitUntilCommand(() -> hopper.indexAtGoal())
                                                .andThen(
                                                    Commands.startEnd(
                                                        () -> initialShots = false,
                                                        () -> initialShots = true))))))),
            Commands.parallel(
                passAim(),
                Commands.waitUntil(
                        () ->
                            hood.atGoal()
                                && drive.getRotation().getDegrees()
                                        - AllianceFlip.apply(
                                                drive
                                                    .getPose()
                                                    .getTranslation()
                                                    .nearest(
                                                        List.of(
                                                            AllianceFlip.apply(
                                                                FieldConstants.Depot
                                                                    .left_far_corner),
                                                            AllianceFlip.flipY(
                                                                AllianceFlip.apply(
                                                                    FieldConstants.Depot
                                                                        .left_far_corner)))))
                                            .minus(drive.getPose().getTranslation())
                                            .getAngle()
                                            .getDegrees()
                                    < 5)
                    .andThen(passShoot())),
            () ->
                ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                        && drive.getPose().getX()
                            <= AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX())
                    || (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                        && drive.getPose().getX()
                            >= AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX()))),
        Commands.parallel(
            manualAim(() -> distance.getAsDouble()),
            new WaitUntilCommand(() -> hood.atGoal())
                .andThen(manualShoot(() -> distance.getAsDouble()))),
        () -> distance.getAsDouble() == 0);
  }

  public static Command staticAim() {
    return Commands.runEnd(
        () ->
            hood.setAngle(
                inclineHueristic(
                    AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getNorm())),
        hood::stop,
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
                                    .getNorm())
                            + ((initialShots) ? 200 : 0)),
                shooter::stop,
                shooter),
            new WaitCommand(0.1)
                .andThen(
                    new WaitUntilCommand(() -> shooter.atGoal())
                        .andThen(
                            Commands.parallel(
                                Commands.runEnd(
                                    () -> hopper.runHopper(9.0, 5000), hopper::stop, hopper),
                                new WaitCommand(0.1)
                                    .andThen(
                                        new WaitUntilCommand(() -> hopper.indexAtGoal())
                                            .andThen(
                                                Commands.startEnd(
                                                    () -> initialShots = false,
                                                    () -> initialShots = true)))))))
        .finallyDo(() -> initialShots = true);
  }

  public static Command manualAim(DoubleSupplier distance) {
    return Commands.runEnd(
        () -> hood.setAngle(inclineHueristic(Units.feetToMeters(distance.getAsDouble()))),
        hood::stop,
        hood);
  }

  public static Command manualShoot(DoubleSupplier distance) {
    return Commands.parallel(
            Commands.runEnd(
                () ->
                    shooter.setVelocityRPM(
                        RPMRegress(Units.feetToMeters(distance.getAsDouble()))
                            + ((initialShots) ? 300 : 0)),
                shooter::stop,
                shooter),
            new WaitCommand(0.1)
                .andThen(
                    new WaitUntilCommand(() -> shooter.atGoal())
                        .andThen(
                            Commands.parallel(
                                Commands.runEnd(
                                    () -> hopper.runHopper(9.0, 5000), hopper::stop, hopper),
                                new WaitCommand(0.1)
                                    .andThen(
                                        new WaitUntilCommand(() -> hopper.indexAtGoal())
                                            .andThen(
                                                Commands.startEnd(
                                                    () -> initialShots = false,
                                                    () -> initialShots = true)))))))
        .finallyDo(() -> initialShots = true);
  }

  public static double RPMRegress(double distance) {
    return 38 * Math.pow((distance - 1.5), 2) + 1800;
  }

  public static Rotation2d inclineHueristic(double distance) {
    return Rotation2d.fromRadians(Math.PI / 2 - Math.atan(7 / distance));
  }

  public static Command passAim() {
    return Commands.runEnd(() -> hood.setAngle(Rotation2d.fromDegrees(40)), hood::stop, hood);
  }

  public static Command passShoot() {
    return Commands.parallel(
        Commands.runEnd(
            () ->
                shooter.setVelocityRPM(
                    110 * Units.metersToFeet(AllianceFlip.apply(drive.getPose()).getX())),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(() -> hopper.runHopper(9.0, 5000), hopper::stop, hopper))));
  }

  public static Command slowUp(AutoCommands.Size size) {
    return Commands.either(
        Commands.sequence(
                new WaitCommand(
                    switch (size) {
                      case PRE -> 0.5;
                      case HALF -> 1.5;
                      case FULL -> 4.0;
                    }),
                Commands.runEnd(() -> wrist.runWristVolts(4), wrist::stop, wrist)
                    .until(() -> wrist.getAngle() > 90))
            .alongWith(Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake)),
        Commands.none(),
        () -> wrist.shakeEnable);
  }

  public static Command downNoStall() {
    return Commands.runEnd(() -> wrist.runWristVolts(-4), wrist::stop, wrist)
        .until(() -> wrist.getAngle() < 30);
  }

  public static Command forceDown() {
    return Commands.sequence(
            Commands.run(() -> wrist.runWristVolts(-6), wrist).withTimeout(0.12),
            Commands.run(() -> wrist.runWristVolts(8), wrist).withTimeout(0.12),
            Commands.run(() -> wrist.runWristVolts(-8), wrist))
        .until(() -> wrist.getAngle() < 30)
        .finallyDo(wrist::stop);
  }

  public static Command goodStow() {
    return Commands.runEnd(() -> wrist.runWristVolts(5), wrist::stop, wrist)
        .until(() -> wrist.getAngle() > 120);
  }
}
