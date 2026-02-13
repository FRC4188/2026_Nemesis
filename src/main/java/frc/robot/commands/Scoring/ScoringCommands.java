package frc.robot.commands.Scoring;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CSPLib.util.ProjMath;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.function.DoubleSupplier;

public class ScoringCommands {

  public static Command WindUp(Launcher launcher, double RPM) {
    return Commands.runOnce(() -> launcher.runShooter(RPM), launcher);
  }

  public static Command aim(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Launcher launcher,
      DoubleSupplier RPM) {

    ProfiledPIDController angleController = Constants.Drive.ANGLE_PID;

    return Commands.run(
            () -> {
              Rotation2d incline;
              Rotation2d azimuth;
              Rotation3d calc;

              if (drive.getChassisSpeeds().vxMetersPerSecond == 0.0
                  && drive.getChassisSpeeds().vyMetersPerSecond == 0.0) {
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
                                72.0 - Constants.ShooterConstants.location.getZ())),
                        new Translation2d(
                                drive.getChassisSpeeds().vxMetersPerSecond,
                                drive.getChassisSpeeds().vyMetersPerSecond)
                            .rotateBy(drive.getRotation()));
              }

              if (calc.getY() == -Math.PI / 2) {
                incline =
                    ProjMath.staticShot(
                        (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam)
                            / 60.0,
                        FieldConstants.Hub.hub_center_2d.minus(
                            new Translation2d(
                                drive.getPose().getTranslation().getNorm(),
                                72.0 - Constants.ShooterConstants.location.getZ())));
                azimuth =
                    FieldConstants.Hub.hub_center_2d
                        .minus(drive.getPose().getTranslation())
                        .getAngle();

              } else {
                incline = new Rotation2d(calc.getY());
                azimuth = new Rotation2d(calc.getZ());
              }

              launcher.setHood(incline);
              launcher.runShooter(RPM.getAsDouble());

              double omega =
                  angleController.calculate(drive.getRotation().getRadians(), azimuth.getRadians())
                      + angleController.getSetpoint().velocity * Constants.Drive.ANGLE_FF;

              if (angleController.atGoal()
                  && angleController.getVelocityTolerance() == 0.0
                  && xSupplier.getAsDouble() == 0.0
                  && ySupplier.getAsDouble() == 0.0) {
                drive.stopWithX();
                return;
              } else if (angleController.atGoal()
                  && angleController.getVelocityTolerance() == 0.0) {
                omega = 0.0;
              }

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                      ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds, AllianceFlip.apply(drive.getRotation())));
            },
            launcher,
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }
}
