package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
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

  public static final double TOF_SECONDS = 1.218;
  private static final double minimumAcceleration = 1;

  public static boolean autonLocked = false;

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

    // double deltaVx = commandedFCSpeeds.vxMetersPerSecond - currentFCSpeeds.vxMetersPerSecond;
    // double deltaVy = commandedFCSpeeds.vyMetersPerSecond - currentFCSpeeds.vyMetersPerSecond;
    // double deltaSpeed = Math.sqrt(deltaVx * deltaVx + deltaVy * deltaVy);

    displaceX = currentFCSpeeds.vxMetersPerSecond * flightTime;
    displaceY = currentFCSpeeds.vyMetersPerSecond * flightTime;

    // if (deltaSpeed <= minimumAcceleration) {
    //   displaceX = currentFCSpeeds.vxMetersPerSecond * flightTime;
    //   displaceY = currentFCSpeeds.vyMetersPerSecond * flightTime;
    // } else {
    //   double accelerationX = Constants.DriveConstants.DRIVE_MAXACC * deltaVx / deltaSpeed;
    //   double accelerationY = Constants.DriveConstants.DRIVE_MAXACC * deltaVy / deltaSpeed;

    //   double timeToReachSpeed = deltaSpeed / Constants.DriveConstants.DRIVE_MAXACC;
    //   double accelerationTime = Math.min(flightTime, timeToReachSpeed);
    //   double velocityTime = Math.max(0, flightTime - timeToReachSpeed);

    //   displaceX =
    //       currentFCSpeeds.vxMetersPerSecond * accelerationTime
    //           + 0.5 * accelerationX * accelerationTime * accelerationTime
    //           + commandedFCSpeeds.vxMetersPerSecond * velocityTime;
    //   displaceY =
    //       currentFCSpeeds.vyMetersPerSecond * accelerationTime
    //           + 0.5 * accelerationY * accelerationTime * accelerationTime
    //           + commandedFCSpeeds.vyMetersPerSecond * velocityTime;

    // }

    return target.plus(
        new Transform2d(
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? -displaceX
                : displaceX,
            DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? -displaceY
                : displaceY,
            new Rotation2d()));
  }

  public static Command dynamicDrive(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Translation2d> target,
      BooleanSupplier dynamicLock,
      BooleanSupplier staticLock) {
    ProfiledPIDController angleController = Constants.DriveConstants.ANGLE_PID;

    return Commands.runEnd(
            () -> {
              double omega = 0.0;
              if (!dynamicLock.getAsBoolean() && !staticLock.getAsBoolean()) {
                omega = omegaSupplier.getAsDouble() * Constants.DriveConstants.ANGLE_MAXVEL;
                drive.acceptVision(true);
              } else {
                ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();
                ChassisSpeeds requestedSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                            xSupplier.getAsDouble() * Constants.DriveConstants.DRIVE_MAXVEL * 0.17,
                            ySupplier.getAsDouble() * Constants.DriveConstants.DRIVE_MAXVEL * 0.17,
                            0),
                        AllianceFlip.apply(drive.getRotation()));

                omega =
                    (dynamicLock.getAsBoolean())
                        ? angleController.calculate(
                            drive.getRotation().getRadians(),
                            lookahead(
                                    new Pose2d(target.get(), new Rotation2d()),
                                    currentSpeeds,
                                    requestedSpeeds,
                                    TOF_SECONDS)
                                .getTranslation()
                                .minus(drive.getPose().getTranslation())
                                .getAngle()
                                .minus(Constants.DriveConstants.local_offset)
                                .getRadians())
                        : angleController.calculate(
                            drive.getRotation().getRadians(),
                            AllianceFlip.apply(target.get())
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
                          * (dynamicLock.getAsBoolean() ? 0.17 : 1.0),
                      ySupplier.getAsDouble()
                          * Constants.DriveConstants.DRIVE_MAXVEL
                          * (dynamicLock.getAsBoolean() ? 0.17 : 1.0),
                      omega);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds, AllianceFlip.apply(drive.getRotation())));
            },
            () -> {
              drive.stopWithX();
              drive.acceptVision(true);
            },
            drive)
        .beforeStarting(
            () ->
                angleController.reset(
                    drive.getRotation().getRadians(),
                    drive.getChassisSpeeds().omegaRadiansPerSecond))
        .alongWith(
            Commands.either(
                dynamicShoot(
                    () ->
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            new ChassisSpeeds(
                                xSupplier.getAsDouble()
                                    * Constants.DriveConstants.DRIVE_MAXVEL
                                    * 0.17,
                                ySupplier.getAsDouble()
                                    * Constants.DriveConstants.DRIVE_MAXVEL
                                    * 0.17,
                                0),
                            AllianceFlip.apply(drive.getRotation())),
                    () -> drive.getChassisSpeeds(),
                    target),
                ScoringCommands.shoot(() -> 0.0),
                () -> dynamicLock.getAsBoolean() || !staticLock.getAsBoolean()));
  }

  public static boolean initialShots = true;

  public static Command dynamicShoot(
      Supplier<ChassisSpeeds> requestedSpeeds,
      Supplier<ChassisSpeeds> currentSpeeds,
      Supplier<Translation2d> target) {
    return Commands.either(
        Commands.parallel(
            Commands.startEnd(() -> autonLocked = true, () -> autonLocked = false),
            Commands.runEnd(
                () ->
                    hood.setAngle(
                        ScoringCommands.inclineHueristic(
                            AllianceFlip.apply(target.get())
                                .minus(drive.getPose().getTranslation())
                                .getNorm())),
                hood::stop,
                hood),
            Commands.runEnd(
                () ->
                    shooter.setVelocityRPM(
                        ScoringCommands.RPMRegress(
                                lookahead(
                                        new Pose2d(target.get(), new Rotation2d()),
                                        currentSpeeds.get(),
                                        requestedSpeeds.get(),
                                        TOF_SECONDS)
                                    .getTranslation()
                                    .minus(drive.getPose().getTranslation())
                                    .getNorm())
                            + ((initialShots) ? 200 : 0)),
                shooter::stop,
                shooter),
            new WaitCommand(0.1)
                .andThen(
                    new WaitUntilCommand(() -> shooter.atGoal() && hood.atGoal())
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
            ScoringCommands.passAim(),
            Commands.waitUntil(() -> hood.atGoal())
                .andThen(
                    Commands.parallel(
                        Commands.runEnd(
                            () ->
                                shooter.setVelocityRPM(
                                    110
                                        * Units.metersToFeet(
                                            AllianceFlip.apply(
                                                    lookahead(
                                                            new Pose2d(
                                                                target.get(), new Rotation2d()),
                                                            currentSpeeds.get(),
                                                            requestedSpeeds.get(),
                                                            TOF_SECONDS)
                                                        .getTranslation()
                                                        .minus(
                                                            FieldConstants.Depot.left_far_corner))
                                                .getX())),
                            shooter::stop,
                            shooter),
                        new WaitCommand(0.1)
                            .andThen(
                                new WaitUntilCommand(() -> shooter.atGoal())
                                    .andThen(
                                        Commands.runEnd(
                                            () -> hopper.runHopper(9.0, 5000),
                                            hopper::stop,
                                            hopper)))))),
        () -> target.get().equals(FieldConstants.Hub.hub_center_2d));
  }
}
