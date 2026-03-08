package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ScoringCommands {
  public static LoggedNetworkNumber _Angle =
      new LoggedNetworkNumber("Aim Tuning/Hood Degrees", 55.0);

  public static Command data(
      Drive drive, Shooter shooter, Hood hood, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
        Commands.run(
            () ->
                Logger.recordOutput(
                    "Aim Tuning/Distance",
                    AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getNorm())),
        DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () ->
                AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                    .minus(drive.getPose().getTranslation())
                    .getAngle()),
        hood.setPosition(() -> Rotation2d.fromDegrees(_Angle.getAsDouble())),
        shooter.setVelocity(
            () ->
                RPMHuersitic(
                    AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getNorm())));
  }

  public static Command aim(
      Drive drive, Shooter shooter, Hood hood, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(
                drive,
                xSupplier,
                ySupplier,
                () ->
                    AllianceFlip.apply(FieldConstants.Hub.hub_aim)
                        .minus(drive.getPose().getTranslation())
                        .getAngle()),
            hood.setPosition(
                () ->
                    inclineRegress(
                        AllianceFlip.apply(FieldConstants.Hub.hub_aim)
                            .minus(drive.getPose().getTranslation())
                            .getNorm())),
            shooter.setVelocity(
                () ->
                    RPMHuersitic(
                        AllianceFlip.apply(FieldConstants.Hub.hub_aim)
                            .minus(drive.getPose().getTranslation())
                            .getNorm())))
        .beforeStarting(drive.disableVision())
        .finallyDo(() -> drive.enableVision().execute());
  }

  public static Rotation2d inclineRegress(double distance) {
    return Rotation2d.fromDegrees(-0.121715 * Math.exp(distance) + 67.74133);
  }

  public static double RPMHuersitic(double distance) {
    return 1500 * (distance / 5.0) + 1500;
  }

  private static Translation2d calcDis = new Translation2d();
  private static double timeAvg = 2.0;

  public static Command shootOnMove(
      Drive drive, Shooter shooter, Hood hood, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
        Commands.run(
            () -> {
              calcDis =
                  AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                      .minus(drive.getPose().getTranslation())
                      .minus(
                          new Translation2d(
                              xSupplier.getAsDouble()
                                  * drive.getMaxLinearSpeedMetersPerSec()
                                  * timeAvg,
                              ySupplier.getAsDouble()
                                  * drive.getMaxLinearSpeedMetersPerSec()
                                  * timeAvg));
            }),
        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> calcDis.getAngle()),
        hood.setPosition(() -> inclineRegress(calcDis.getNorm())),
        shooter.setVelocity(() -> RPMHuersitic(calcDis.getNorm())));
  }

  public static Command passing(
      Drive drive, Shooter shooter, Hood hood, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              return (drive
                      .getPose()
                      .getTranslation()
                      .nearest(
                          List.of(
                              AllianceFlip.apply(FieldConstants.Bump.left_bump_alliance_entrance),
                              AllianceFlip.apply(
                                  FieldConstants.Bump.right_bump_alliance_entrance))))
                  .minus(drive.getPose().getTranslation())
                  .getAngle();
            }),
        hood.setPosition(() -> Rotation2d.fromDegrees(35)),
        shooter.setVelocity(() -> 3000.0));
  }

  public static Command shootFor(
      double seconds, Drive drive, Shooter shooter, Hood hood, Hopper hopper, DoubleSupplier RPM) {

    return Commands.sequence(
        ScoringCommands.aim(
                drive,
                shooter,
                hood,
                () -> drive.getChassisSpeeds().vxMetersPerSecond,
                () -> drive.getChassisSpeeds().vyMetersPerSecond)
            .beforeStarting(drive.disableVision()),
        new WaitUntilCommand(() -> (hood.atGoal() && shooter.atGoal())),
        hopper.runVolts(() -> 0.5, () -> 0.5).withTimeout(seconds),
        Commands.runOnce(() -> drive.enableVision()));
  }

  public static Command shootUntil(
      BooleanSupplier disable,
      Drive drive,
      Shooter shooter,
      Hood hood,
      Hopper hopper,
      DoubleSupplier RPM) {

    return Commands.sequence(
        ScoringCommands.aim(
                drive,
                shooter,
                hood,
                () -> drive.getChassisSpeeds().vxMetersPerSecond,
                () -> drive.getChassisSpeeds().vyMetersPerSecond)
            .beforeStarting(drive.disableVision()),
        new WaitUntilCommand(() -> (hood.atGoal() && shooter.atGoal())),
        hopper.runVolts(() -> 0.5, () -> 0.5).until(disable),
        Commands.runOnce(() -> drive.enableVision()));
  }
}
