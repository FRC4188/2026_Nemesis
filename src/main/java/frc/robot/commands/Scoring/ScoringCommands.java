package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import frc.robot.util.SOTM;
import frc.robot.util.ShotCalc;
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

  public static LoggedNetworkNumber apexHeight =
      new LoggedNetworkNumber("Aim Tuning/Apex Height Inches", Units.inchesToMeters(102));

  public static LoggedNetworkNumber targetHeight =
      new LoggedNetworkNumber("Aim Tuning/Target Height Inches", Units.inchesToMeters(72));
  public static LoggedNetworkNumber passHeight =
      new LoggedNetworkNumber("Aim Tuning/Pass Height Inches", Units.inchesToMeters(120));

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
    return Commands.sequence(
        new WaitCommand(1.5), // wait
        Commands.parallel(
            Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake),
            Commands.repeatingSequence(
                    Commands.runEnd(() -> wrist.runWristVolts(1.15), wrist::stop, wrist)
                        .until(
                            () ->
                                wrist.getStatorCurrent()
                                    > Constants.WristConstants.fuelStatorCurrent)
                        .andThen(
                            Commands.either(
                                Commands.runEnd(() -> wrist.runWristVolts(-2), wrist::stop, wrist)
                                    .withTimeout(0.6),
                                Commands.runEnd(
                                    () -> wrist.runWristVolts(-0.5), wrist::stop, wrist),
                                () -> wrist.getAngle() > 90)))
                .onlyWhile(() -> wrist.getAngle() < 110)));
  }

  public static Command wrist1678() {
    return Commands.sequence(
        new WaitCommand(1.5), // wait
        Commands.parallel(
            Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake),
            Commands.repeatingSequence(
                    Commands.runEnd(() -> wrist.runWristVolts(1.15), wrist::stop, wrist)
                        .until(
                            () ->
                                wrist.getStatorCurrent()
                                    > Constants.WristConstants.fuelStatorCurrent)
                        .andThen(lowerIntakeCoast()))
                .onlyWhile(() -> wrist.getAngle() < 110)));
  }

  // yea this doesn't actually work. explanation later
  // akhil here : explanation is it doesnt work

  public static Command lowerIntakeTorque() {
    return Commands.runEnd(() -> wrist.runWristVolts(-4.5), wrist::stop, wrist)
        .until(() -> wrist.getStatorCurrent() > Constants.WristConstants.bumperStatorCurrent)
        .alongWith(Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake));
  }

  // FOR TESTING
  public static Command lowerIntakeCoast() {
    return Commands.sequence(
        Commands.runEnd(() -> wrist.runWristVolts(-8), wrist::stop).withTimeout(0.15),
        Commands.runEnd(() -> wrist.setCoastVolts(-3.5), wrist::stop, wrist)
            .until(() -> Math.abs(wrist.getVelocityDegPerSec()) < 0.007)
            .finallyDo(
                () -> {
                  wrist.zero();
                  wrist.stop();
                }));
  }

  public static Command lowerIntakeBang() {
    return Commands.sequence(
        Commands.runEnd(() -> wrist.runWristVolts(-10), wrist::stop).withTimeout(0.15),
        Commands.runEnd(() -> wrist.setCoastVolts(-4), wrist::stop, wrist)
            .until(() -> Math.abs(wrist.getVelocityDegPerSec()) < 0.007)
            .finallyDo(
                () -> {
                  wrist.zero();
                  wrist.stop();
                }));
  }

  public static Command intake() {
    return Commands.either(forceDown(), lowerIntakeCoast(), () -> wrist.getAngle() > 85)
        .alongWith(Commands.runEnd(() -> intake.intakeVolts(8.75), intake::stop, intake))
        .finallyDo(
            () -> {
              wrist.stop();
              intake.stop();
              wrist.zero();
            });
  }

  public static Command lowSpinShooter() {
    return Commands.runEnd(() -> shooter.setVelocityRPM(500), () -> shooter.runTC(0), shooter)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public static Command shooterIntake() {
    return Commands.parallel(
            Commands.runEnd(() -> shooter.setVelocityRPM(-800), shooter::stop, shooter),
            Commands.runEnd(() -> hopper.runHopper(0, -2000), hopper::stop, hopper))
        .withTimeout(0.5)
        .finallyDo(
            () -> {
              shooter.stop();
              hopper.stop();
            });
  }

  public static Command toggleWristCompress(Trigger intaking) {
    return Commands.repeatingSequence(
        Commands.waitUntil(intaking.negate()), wristCompress().until(intaking).asProxy());
  }

  public static boolean initialShots = true;

  public static Command shoot(DoubleSupplier distance, Trigger intaking) {
    return Commands.either( // either for manual or vision shooting
        Commands.either( // either for static or pass shooting
            Commands.parallel( // static shooting
                staticAim(), // aim at hub
                lowSpinShooter() // spin up shooter
                    .until(
                        () ->
                            hood.atGoal()
                                && (Constants.DriveConstants.ANGLE_PID.atGoal()
                                    || drive.getTranslationalSpeed()
                                        > 1e-9)) // wait until hood and drive are at goal
                    .andThen( // then shoot
                        Commands.parallel(
                            staticShoot(), // shoot
                            Commands.waitUntil(() -> shooter.atGoal())
                                .andThen(
                                    Commands.waitSeconds(0.1),
                                    toggleWristCompress(
                                        intaking)) // wait until shooter is at goal and then toggle
                            // wrist compress
                            ))),
            Commands.parallel( // pass shooting
                passAim(), // aim at hub
                lowSpinShooter()
                    .until(() -> hood.atGoal())
                    .andThen( // wait until hood is at max angle
                        Commands.parallel( // then shoot
                            passShoot(), // shoot
                            Commands.waitUntil(() -> shooter.atGoal())
                                .andThen(
                                    Commands.waitSeconds(0.1),
                                    toggleWristCompress(
                                        intaking)) // wait until shooter is at goal and then toggle
                            // wrist compress
                            ))),
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
                .andThen(
                    Commands.parallel(
                        manualShoot(() -> distance.getAsDouble()),
                        Commands.waitUntil(() -> shooter.atGoal())
                            .andThen(Commands.waitSeconds(0.1), toggleWristCompress(intaking))))),
        () -> distance.getAsDouble() == 0);
  }

  public static Command staticAim() {
    return Commands.runEnd(
        () ->
            hood.setAngle(
                ShotCalc.getShotAngle(
                    SOTM.lookahead(
                            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d),
                            drive.getChassisSpeeds(),
                            ShotCalc.getTimeOfFlight(
                                apexHeight.getAsDouble(), targetHeight.getAsDouble()))
                        .minus(drive.getPose().getTranslation())
                        .getNorm(),
                    apexHeight.getAsDouble(),
                    targetHeight.getAsDouble())),
        hood::stop,
        hood);
  }

  public static double getRegressionRPM() {
    return RPMRegress(
        AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
            .minus(drive.getPose().getTranslation())
            .getNorm());
  }

  public static double getRegressionAngle() {
    return inclineHueristic(
            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                .minus(drive.getPose().getTranslation())
                .getNorm())
        .getDegrees();
  }

  public static Command staticShoot() {
    return Commands.parallel(
            Commands.runEnd(
                () ->
                    shooter.setVelocityRPM(
                        ShotCalc.getShotRPM(
                                SOTM.lookahead(
                                        AllianceFlip.apply(FieldConstants.Hub.hub_center_2d),
                                        drive.getChassisSpeeds(),
                                        ShotCalc.getTimeOfFlight(
                                            apexHeight.getAsDouble(), targetHeight.getAsDouble()))
                                    .minus(drive.getPose().getTranslation())
                                    .getNorm(),
                                apexHeight.getAsDouble(),
                                targetHeight.getAsDouble())
                            + ((initialShots)
                                ? 200
                                : (initialShots
                                        && AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                                .getDistance(drive.getPose().getTranslation())
                                            >= 4)
                                    ? 300
                                    : 0)),
                shooter::stop,
                shooter),
            new WaitCommand(0.1)
                .andThen(
                    new WaitUntilCommand(() -> shooter.atGoal())
                        .andThen(new WaitCommand(0.5))
                        .andThen(
                            Commands.parallel(
                                Commands.runEnd(
                                    () -> hopper.runHopper(7.0, 2000),
                                    hopper::stop,
                                    hopper), // 5000
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
                            + ((initialShots)
                                ? 300 - hopper.getIndexerSpeed() * 300 / 5000
                                : 0)), // initial rpm
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
                    ShotCalc.getShotRPM(
                        Units.metersToFeet(
                            AllianceFlip.apply(drive.getPose()).getX()
                                - Units.inchesToMeters(156.61)
                                + Units.feetToMeters(2)),
                        Units.inchesToMeters(passHeight.getAsDouble()),
                        Units.inchesToMeters(0))),
            shooter::stop,
            shooter),
        new WaitCommand(0.1)
            .andThen(
                new WaitUntilCommand(() -> shooter.atGoal())
                    .andThen(
                        Commands.runEnd(() -> hopper.runHopper(9.0, 2000), hopper::stop, hopper))));
  }

  // Units.metersToFeet(AllianceFlip.apply(drive.getPose()).getX())
  //   public static Command slowUp(AutoCommands.Size size) {
  //     return Commands.either(
  //         Commands.sequence(
  //                 new WaitCommand(
  //                     switch (size) {
  //                       case PRE -> 0.5;
  //                       case HALF -> 1.5;
  //                       case FULL -> 4.0;
  //                     }),
  //                 Commands.runEnd(() -> wrist.runWristVolts(4), wrist::stop, wrist)
  //                     .until(() -> wrist.getAngle() > 90))
  //             .alongWith(Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake)),
  //         Commands.none(),
  //         () -> wrist.shakeEnable);
  //   }

  public static Command downNoStall() {
    return Commands.runEnd(() -> wrist.runWristVolts(-4), wrist::stop, wrist)
        .until(() -> wrist.getAngle() < 30);
  }

  public static Command forceDown() {
    return Commands.sequence(
            Commands.run(() -> wrist.runWristVolts(-8), wrist).withTimeout(0.12),
            Commands.run(() -> wrist.runWristVolts(8), wrist).withTimeout(0.12),
            Commands.run(() -> wrist.runWristVolts(-8), wrist))
        .until(
            () ->
                wrist.getAngle() < 30
                    || wrist.getStatorCurrent() > Constants.WristConstants.bumperStatorCurrent)
        .finallyDo(wrist::stop);
  }

  public static Command goodStow() {
    return Commands.runEnd(() -> wrist.runWristVolts(5), wrist::stop, wrist)
        .until(() -> wrist.getAngle() > 120);
  }
}
