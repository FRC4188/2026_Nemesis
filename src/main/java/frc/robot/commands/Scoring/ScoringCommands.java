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
    return Commands.runOnce(wrist::zero, wrist)
        .andThen(
            Commands.parallel(
                Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake),
                Commands.repeatingSequence(
                        Commands.runEnd(() -> wrist.runWristVolts(1.9), wrist::stop, wrist)
                            .until(
                                () ->
                                    wrist.getStatorCurrent()
                                        > Constants.WristConstants.fuelStatorCurrent)
                            .andThen(
                                Commands.either(
                                    Commands.runEnd(
                                            () -> wrist.runWristVolts(-2), wrist::stop, wrist)
                                        .withTimeout(0.6),
                                    Commands.waitSeconds(0.3),
                                    () -> wrist.getAngle() > 90)))
                    .onlyWhile(() -> wrist.getAngle() < 110)));
  }

  public static Command lowerIntakeTorque() {
    return Commands.runEnd(() -> wrist.runWristVolts(-4.5), wrist::stop, wrist)
        .until(() -> wrist.getStatorCurrent() > Constants.WristConstants.bumperStatorCurrent)
        .alongWith(Commands.runEnd(() -> intake.intakeVolts(5.0), intake::stop, intake));
  }

  public static Command intake() {
    return Commands.either(
            forceDown(), lowerIntakeTorque(), () -> wrist.getAngle() > 100)
        .alongWith(Commands.runEnd(() -> intake.intakeVolts(8.75), intake::stop, intake))
        .finallyDo(
            () -> {
              wrist.stop();
              intake.stop();
              wrist.zero();
            });
  }

  public static Command lowSpinShooter() {
    return Commands.runEnd(() -> shooter.setVelocityRPM(500), shooter::idle, shooter)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public static Command shooterIntake() {
    return Commands.parallel(
        Commands.runEnd(() -> shooter.setVelocityRPM(-800), shooter::stop, shooter),
        Commands.runEnd(() -> hopper.runHopper(0, -2000), hopper::stop, hopper)
    ).withTimeout(0.5).finallyDo(() -> {
      shooter.stop();
      hopper.stop();
    });
  }

  public static Command toggleWristCompress(Trigger intaking) {
    return Commands.repeatingSequence(
        Commands.waitUntil(intaking.negate()),
        wristCompress()
            .until(intaking)
            .asProxy()
    );
}

  public static boolean initialShots = true;

  /*
   * FOR DEBUGGING 9/2/2026
   * Work on tuning volts and fuel stator current for wrist compress
   * Find the bumper stator current for force down and lower intake torque, setting volts low for both
   * Find the time it takes for hood to start to go up, and for the first ball to get to the shooter, 
   * record time and tell me between interviews
   * 
   * IF EVERYTHING IS DONE: then do me a favor:
   * Do several recordings of the robot shooting from the robot shooting from different distances, 
   * and in the same way as data shoot, record the distance from the hub before you shoot, and then 
   * the time of flight for each ball. i need a 240 fps slow mo camera to do this, Ansh if you could
   * "borrow" Ishaan's phone for a bit. (17 pros have a 240 fps slow mo camera). after you figure out
   * time of flight for a SINGULAR ball, record the distance from the hub and the time of flight in Desmos, 
   * and then do this for several distances (0.5, 0.25, 0.75, 1, 1.5, etc.) i need roughly 25-35 different distances
   * for better results (go big or go home). Find the best fit line for the data and send it to me on slack.
   * 
   * measure from the first frame the ball leaves the shooter, to the frame it hits the top of the hub.
   * for better accuracy, count the number of frames instead of the seconds it shows on the bottom of the video, 
   * and then divide by 240 to get the time in seconds.
   * 
   * 
   * DELETE AFTER YOU ARE DONE WITH THIS, I WILL NOT BE ABLE TO HELP YOU WITH THIS, I WILL BE IN INTERVIEWS
   */

  public static Command shoot(DoubleSupplier distance, Trigger intaking) {
    return Commands.either( // either for manual or vision shooting
        Commands.either( // either for static or pass shooting
            Commands.parallel( // static shooting
                staticAim(), // aim at hub
                lowSpinShooter() // spin up shooter
                    .until(() -> hood.atGoal() && Constants.DriveConstants.ANGLE_PID.atGoal()) // wait until hood and drive are at goal
                    .andThen( // then shoot
                        Commands.parallel(
                            staticShoot(), // shoot
                            Commands.waitUntil(() -> shooter.atGoal()).andThen(Commands.waitSeconds(0.1), toggleWristCompress(intaking)) // wait until shooter is at goal and then toggle wrist compress
                            ))),
            Commands.parallel( // pass shooting
                passAim(), // aim at hub
                lowSpinShooter().until(() -> hood.getAngle() > hood.maxAngle()).andThen( // wait until hood is at max angle
                Commands.parallel( // then shoot
                passShoot(), // shoot
                Commands.waitUntil(() -> shooter.atGoal()).andThen(Commands.waitSeconds(0.1), toggleWristCompress(intaking)) // wait until shooter is at goal and then toggle wrist compress
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
                        Commands.waitUntil(() -> shooter.atGoal()).andThen(Commands.waitSeconds(0.1), toggleWristCompress(intaking))
                    )
                )),
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
