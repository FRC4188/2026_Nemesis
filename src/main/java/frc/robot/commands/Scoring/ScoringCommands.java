package frc.robot.commands.Scoring;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CSPLib.util.ProjMath;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;

public class ScoringCommands {

  private static Rotation3d calc;

  public static Command aim(
      Drive drive,
      Shooter shooter,
      Hood hood,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier RPM) {

    return Commands.parallel(
        Commands.run(
            () -> {
              if (xSupplier.getAsDouble() == 0.0 && ySupplier.getAsDouble() == 0.0) {
                calc = new Rotation3d(0.0, -Math.PI / 2, 0.0);
              } else {
                calc =
                    ProjMath.movingShot(
                        (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam)
                            / 60.0,
                        new Translation3d(
                            AllianceFlip.flipX(FieldConstants.Hub.hub_center_2d.getX())
                                - drive.getPose().getTranslation().getX(),
                            AllianceFlip.flipY(FieldConstants.Hub.hub_center_2d.getY())
                                - drive.getPose().getTranslation().getY(),
                            Units.inchesToMeters(
                                Units.inchesToMeters(72.0)
                                    - Constants.ShooterConstants.location.getZ())),
                        new Translation2d(
                                drive.getChassisSpeeds().vxMetersPerSecond,
                                drive.getChassisSpeeds().vyMetersPerSecond)
                            .rotateBy(drive.getRotation()));
              }
            }),
        hood.setPosition(
            () -> {
              if (calc.getY() == -Math.PI / 2) {
                return ProjMath.staticShot(
                    (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam) / 60.0,
                    new Translation2d(
                        AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                            .minus(drive.getPose().getTranslation())
                            .getNorm(),
                        Units.inchesToMeters(72.0) - Constants.ShooterConstants.location.getZ()));
              }
              return new Rotation2d(calc.getY());
            }),
        DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              if (calc.getY() == -Math.PI / 2) {
                return AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                    .minus(drive.getPose().getTranslation())
                    .getAngle();
              }
              return new Rotation2d(calc.getZ());
            }),
        shooter.setVelocity(RPM));
  }

  public static Command shake(Wrist wrist) {
    return Commands.repeatingSequence(
        wrist.runWrist(() -> 1.0).withTimeout(0.5),
        new WaitCommand(0.5).until(() -> wrist.getAngle() > Units.degreesToRadians(130)));
  }
}
