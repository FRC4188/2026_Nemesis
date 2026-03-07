package frc.robot.commands.Scoring;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.CSPLib.util.ProjMath;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.climber.Climber;
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

public class ScoringCommands {

  private static Rotation3d calc = new Rotation3d(0, -Math.PI / 2, 0);

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
              // if (xSupplier.getAsDouble() == 0.0 && ySupplier.getAsDouble() == 0.0) {
              //   calc = new Rotation3d(0.0, -Math.PI / 2, 0.0);
              // } else {
              //   Translation2d relativeGoal =
              //       AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
              //           .minus(drive.getPose().getTranslation());

              //   relativeGoal =
              //       relativeGoal.minus(
              //           new Translation2d(Constants.ShooterConstants.location.getX(), 0.0)
              //               .rotateBy(relativeGoal.getAngle()));

              //   calc =
              //       ProjMath.movingShot(
              //           (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam)
              //               / 60.0,
              //           new Translation3d(
              //               relativeGoal.getX(),
              //               relativeGoal.getY(),
              //               Units.inchesToMeters(
              //                   Units.inchesToMeters(72.0)
              //                       - Constants.ShooterConstants.location.getZ())),
              //           new Translation2d(
              //               xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
              //               ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec()));
              // }
            }),
        hood.setPosition(
            () -> {
              Logger.recordOutput(
                  "Aim Tuning/Distance",
                  AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                          .minus(drive.getPose().getTranslation())
                          .getNorm()
                      - Constants.ShooterConstants.location.getX());

              if (calc.getY() == -Math.PI / 2) {
                Rotation2d incline =
                    ProjMath.staticShot(
                        (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam)
                            / 60.0,
                        new Translation2d(
                            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                    .minus(drive.getPose().getTranslation())
                                    .getNorm()
                                - Constants.ShooterConstants.location.getX(),
                            Units.inchesToMeters(72.0)
                                - Constants.ShooterConstants.location.getZ()));
                if (incline.getDegrees() < -45.0) return Rotation2d.fromDegrees(45);
                Logger.recordOutput("Aim Tuning/Incline Angle", incline.getDegrees());
                return incline;
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
        shooter.setVelocity(() -> (RPM.getAsDouble() * 3.0)));
  }

  // TODO: Add the pose for climbing
  public static Command goToClimb(Drive drive, Climber climber) {
    return Commands.sequence(
        new DriveToPose(drive, () -> drive.getPose().nearest(null)), // lineup
        climber.raise(),
        new WaitUntilCommand(() -> climber.atGoal()),
        new DriveToPose(drive, null) // placement
        );
  }

  public static Command passing(
      Drive drive,
      Shooter shooter,
      Hood hood,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier RPM) {
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
        hood.setPosition(() -> Rotation2d.fromDegrees(45)),
        shooter.setVelocity(() -> (RPM.getAsDouble() + Constants.ShooterConstants.kDropVel)));
  }

  public static Command shootFor(
      double seconds, Drive drive, Shooter shooter, Hood hood, Hopper hopper, DoubleSupplier RPM) {

    return Commands.sequence(
        ScoringCommands.aim(
                drive,
                shooter,
                hood,
                () -> drive.getChassisSpeeds().vxMetersPerSecond,
                () -> drive.getChassisSpeeds().vyMetersPerSecond,
                () -> Constants.ShooterConstants.kMiddleVel)
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
                () -> drive.getChassisSpeeds().vyMetersPerSecond,
                () -> Constants.ShooterConstants.kMiddleVel)
            .beforeStarting(drive.disableVision()),
        new WaitUntilCommand(() -> (hood.atGoal() && shooter.atGoal())),
        hopper.runVolts(() -> 0.5, () -> 0.5).until(disable),
        Commands.runOnce(() -> drive.enableVision()));
  }

  // public static Command shoot(
  //   Shooter shooter,
  //   Hood hood, Hopper hopper,
  //   DoubleSupplier RPM,
  //   BooleanSupplier end) {

  //   return Commands.sequence(
  //     Commands.runOnce(() -> shooter.setVelocity(Constants.ShooterConstants.kMiddleVel)),
  //       new WaitUntilCommand(() -> (hood.atGoal() && shooter.atGoal())),
  //       hopper.runVolts(() -> 0.5, () -> 0.5).until(end)
  //       );
  // }

  // public static Command shootFor(
  //   double seconds, Shooter shooter, Hood hood, Hopper hopper, DoubleSupplier RPM
  // ) {
  //   return Commands.sequence(
  //       new WaitUntilCommand(() -> (hood.atGoal() && shooter.atGoal())),
  //       hopper.runVolts(() -> 0.5, () -> 0.5).withTimeout(seconds));
  // }
}
