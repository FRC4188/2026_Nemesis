package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SOTM { // Experimental Class for Offseason
  private static final Shooter shooter = Shooter.getInstance();
  private static final Hopper hopper = Hopper.getInstance();
  private static final Drive drive = Drive.getInstance();
  private static final Hood hood = Hood.getInstance();
  private static final Wrist wrist = Wrist.getInstance();
  private static final Intake intake = Intake.getInstance();

  private static final double TOF_SECONDS = 1.218;
  private static final double minimumAcceleration = 1;

  public static Pose2d lookahead(
      Pose2d target,
      ChassisSpeeds currentSpeeds,
      ChassisSpeeds commandedSpeeds,
      double flightTime) {
    ChassisSpeeds currentFCSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            currentSpeeds, AllianceFlip.apply(drive.getRotation()));
    ChassisSpeeds commandedFCSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            commandedSpeeds, AllianceFlip.apply(drive.getRotation()));

    double displaceX;
    double displaceY;

    double deltaVx = commandedFCSpeeds.vxMetersPerSecond - currentFCSpeeds.vxMetersPerSecond;
    double deltaVy = commandedFCSpeeds.vyMetersPerSecond - currentFCSpeeds.vyMetersPerSecond;
    double deltaSpeed = Math.sqrt(deltaVx * deltaVx + deltaVy * deltaVy);

    if (deltaSpeed <= 1) {
      displaceX = currentFCSpeeds.vxMetersPerSecond * flightTime;
      displaceY = currentFCSpeeds.vyMetersPerSecond * flightTime;
    } else {
      double accelerationX = Constants.DriveConstants.DRIVE_MAXACC * deltaVx / deltaSpeed;
      double accelerationY = Constants.DriveConstants.DRIVE_MAXACC * deltaVy / deltaSpeed;

      double timeToReachSpeed = deltaSpeed / Constants.DriveConstants.DRIVE_MAXACC;
      double accelerationTime = Math.min(flightTime, timeToReachSpeed);
      double velocityTime = Math.max(0, flightTime - timeToReachSpeed);

      displaceX =
          currentFCSpeeds.vxMetersPerSecond * accelerationTime
              + 0.5 * accelerationX * accelerationTime * accelerationTime
              + commandedFCSpeeds.vxMetersPerSecond * velocityTime;
      displaceY =
          currentFCSpeeds.vyMetersPerSecond * accelerationTime
              + 0.5 * accelerationY * accelerationTime * accelerationTime
              + commandedFCSpeeds.vyMetersPerSecond * velocityTime;
    }

    return target.plus(new Transform2d(-displaceX, -displaceY, new Rotation2d()));
  }

  public static Command dynamicDrive(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Translation2d> target,
      BooleanSupplier lock) {
    ProfiledPIDController angleController = Constants.DriveConstants.ANGLE_PID;

    return Commands.runEnd(
        () -> {
          double omega = 0.0;
          if (!lock.getAsBoolean()) {
            omega = omegaSupplier.getAsDouble() * Constants.DriveConstants.ANGLE_MAXVEL;
            drive.acceptVision(true);
          } else {
            ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();
            ChassisSpeeds requestedSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        xSupplier.getAsDouble() * 0.4, ySupplier.getAsDouble() * 0.4, 0),
                    AllianceFlip.apply(drive.getRotation()));

            omega =
                angleController.calculate(
                    drive.getRotation().getRadians(),
                    AllianceFlip.apply(
                            lookahead(
                                    new Pose2d(target.get(), new Rotation2d()),
                                    currentSpeeds,
                                    requestedSpeeds,
                                    TOF_SECONDS)
                                .getTranslation())
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .minus(Constants.DriveConstants.local_offset)
                        .getRadians());

            if (angleController.atGoal()) {
              if (xSupplier.getAsDouble() == 0.0 && ySupplier.getAsDouble() == 0.0) {
                drive.stopWithX();
                drive.acceptVision(false);
                return;
              }
              omega = 0.0;
              drive.acceptVision(false);
            } else {
              drive.acceptVision(true);
            }
          }

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble()
                      * Constants.DriveConstants.DRIVE_MAXVEL
                      * (lock.getAsBoolean() ? 0.4 : 1.0),
                  ySupplier.getAsDouble()
                      * Constants.DriveConstants.DRIVE_MAXVEL
                      * (lock.getAsBoolean() ? 0.4 : 1.0),
                  omega);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, AllianceFlip.apply(drive.getRotation())));
        },
        () -> {
          drive.stopWithX();
          drive.acceptVision(true);
        });
  }

  public static Command dynamicShoot(
      ChassisSpeeds requestedSpeeds, ChassisSpeeds currentSpeeds, DoubleSupplier distance) {
    return Commands.none();
  }
}
