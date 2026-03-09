package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
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
        Commands.runEnd(
            () -> hood.setShotAngle(Rotation2d.fromDegrees(_Angle.getAsDouble())),
            hood::stow,
            hood),
        Commands.runEnd(
            () ->
                shooter.setVelocityRPM(
                    RPMHuersitic(
                        AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                            .minus(drive.getPose().getTranslation())
                            .getNorm())),
            shooter::stop,
            shooter));
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
            Commands.runEnd(
                () ->
                    hood.setShotAngle(
                        inclineRegress(
                            AllianceFlip.apply(FieldConstants.Hub.hub_aim)
                                .minus(drive.getPose().getTranslation())
                                .getNorm())),
                hood::stow,
                hood),
            Commands.runEnd(
                () ->
                    shooter.setVelocityRPM(
                        RPMHuersitic(
                            AllianceFlip.apply(FieldConstants.Hub.hub_aim)
                                .minus(drive.getPose().getTranslation())
                                .getNorm())),
                shooter::stop,
                shooter))
        .beforeStarting(Commands.runOnce(() -> drive.acceptVision(false)))
        .finallyDo(() -> drive.acceptVision(true));
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
            DriveCommands.joystickDriveAtAngle(
                drive, xSupplier, ySupplier, () -> calcDis.getAngle()),
            Commands.runEnd(
                () -> hood.setShotAngle(inclineRegress(calcDis.getNorm())), hood::stow, hood),
            Commands.runEnd(
                () -> shooter.setVelocityRPM(RPMHuersitic(calcDis.getNorm())),
                shooter::stop,
                shooter))
        .beforeStarting(Commands.runOnce(() -> drive.acceptVision(false)))
        .finallyDo(() -> drive.acceptVision(true));
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
                                  AllianceFlip.apply(
                                      FieldConstants.Bump.left_bump_alliance_entrance),
                                  AllianceFlip.apply(
                                      FieldConstants.Bump.right_bump_alliance_entrance))))
                      .minus(drive.getPose().getTranslation())
                      .getAngle();
                }),
            Commands.runEnd(
                () -> hood.setShotAngle(Rotation2d.fromDegrees(35.0)), hood::stow, hood),
            Commands.runEnd(() -> shooter.setVelocityRPM(3000), shooter::stop, shooter))
        .beforeStarting(Commands.runOnce(() -> drive.acceptVision(false)))
        .finallyDo(() -> drive.acceptVision(true));
  }

  public static Command shake(Wrist wrist) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> wrist.runWristTC(20.0)),
            new WaitUntilCommand(() -> wrist.isStalled()),
            Commands.runOnce(wrist::stop),
            new WaitCommand(0.3))
        .until(() -> wrist.getAngle() > 120.0)
        .finallyDo(() -> wrist.stow());
  }

  public static Command downWoStall(Wrist wrist) {
    return Commands.sequence(
            Commands.runOnce(wrist::down, wrist),
            new WaitUntilCommand(() -> wrist.getAngle() < 45.0 || wrist.isStalled()))
        .finallyDo(wrist::stop);
  }
}
