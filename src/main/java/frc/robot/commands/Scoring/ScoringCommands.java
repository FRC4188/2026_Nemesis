package frc.robot.commands.Scoring;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.function.DoubleSupplier;

public class ScoringCommands {

  public static Rotation3d calc = new Rotation3d(0, -Math.PI / 2, 0);

  public static Command aim(
      Drive drive,
      Shooter shooter,
      Hood hood,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier RPM) {

    return Commands.parallel(
        // Commands.run(
        //     () -> {
        //       if (xSupplier.getAsDouble() == 0.0 && ySupplier.getAsDouble() == 0.0) {
        //         calc = new Rotation3d(0.0, -Math.PI / 2, 0.0);
        //       } else {
        //         calc =
        //             ProjMath.movingShot(
        //                 (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam)
        //                     / 60.0,
        //                 new Translation3d(
        //                     AllianceFlip.flipX(FieldConstants.Hub.hub_center_2d.getX())
        //                         - drive.getPose().getTranslation().getX(),
        //                     AllianceFlip.flipY(FieldConstants.Hub.hub_center_2d.getY())
        //                         - drive.getPose().getTranslation().getY(),
        //                     Units.inchesToMeters(
        //                         Units.inchesToMeters(72.0)
        //                             - Constants.ShooterConstants.location.getZ())),
        //                 new Translation2d(
        //                         drive.getChassisSpeeds().vxMetersPerSecond,
        //                         drive.getChassisSpeeds().vyMetersPerSecond)
        //                     .rotateBy(drive.getRotation()));
        //       }
        //       Logger.recordOutput("Incline", calc.getY());
        //       Logger.recordOutput("Azimuth", calc.getZ());
        //     }),
        hood.setPosition(
            () -> {
              // if (calc.getY() == -Math.PI / 2) {
              // Rotation2d incline =
              //     ProjMath.staticShot(
              //         (RPM.getAsDouble() * Math.PI * Constants.ShooterConstants.kWheelDiam) /
              // 60.0,
              //         new Translation2d(
              //             AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
              //                 .minus(drive.getPose().getTranslation())
              //                 .getNorm(),
              //             Units.inchesToMeters(72.0) -
              // Constants.ShooterConstants.location.getZ()));
              // if (incline.getDegrees() == 90.0) {
              //   return incline;
              // }
              // return incline;
              return Rotation2d.fromDegrees(55.0);
              // }
              // return new Rotation2d(calc.getY());
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
        wrist.runWrist(() -> 2.0).withTimeout(0.5),
        new WaitCommand(0.25).until(() -> wrist.getAngle() > Units.degreesToRadians(130)));
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
        shooter.setVelocity(RPM));
  }
}
