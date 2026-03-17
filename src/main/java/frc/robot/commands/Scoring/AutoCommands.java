package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;

public class AutoCommands {

  public static Command autoShoot(
      Drive drive, Intake intake, Hood hood, Shooter shooter, Hopper hopper, Wrist wrist) {
    return Commands.parallel(
            DriveCommands.autonAtAngle(
                drive,
                () ->
                    AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getAngle()),
            Commands.runEnd(() -> intake.intakeVolts(1.5), () -> intake.stop()).withTimeout(1),
            ScoringCommands.staticAim(drive, hood),
            new WaitCommand(0.1)
                .andThen(new WaitUntilCommand(() -> shooter.atGoal()))
                .andThen(ScoringCommands.staticShoot(drive, shooter, hopper)),
            new WaitCommand(2.5).andThen(ScoringCommands.goodStow(wrist)))
        .withTimeout(6.0);
  }
}
