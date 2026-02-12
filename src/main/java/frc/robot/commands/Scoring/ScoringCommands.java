package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.Loader.Intake.Intake;
import frc.robot.subsystems.Loader.Wrist.Wrist;
import frc.robot.subsystems.Transfer.Hopper.Hopper;
import frc.robot.subsystems.Transfer.Indexer.Indexer;

public class ScoringCommands {

  // redundant example
  public static Command WindUp(Launcher launcher, double RPM) {
    return Commands.runOnce(() -> launcher.runShooter(RPM), launcher);
  }

  public static Command wristDown(Wrist wrist) {
    return Commands.runOnce(() -> wrist.setPosition(new Rotation2d(Constants.WristConstants.Min_A)))
        .andThen(Commands.runOnce(() -> wrist.stop()));
  }

  public static Command wristUp(Wrist wrist) {
    return Commands.runOnce(
        () -> wrist.setPosition(new Rotation2d(Constants.WristConstants.Max_A)));
  }

  public static Command intake(Wrist wrist, Intake intake) {
    return wristDown(wrist).alongWith(Commands.run(() -> wrist.runVolts(8.5)));
  }

  public static Command feed(Hopper hopper, Indexer indexer) {
    return Commands.parallel(
        Commands.run(() -> hopper.runVolts(9)), Commands.run(() -> indexer.runVolts(9)));
  }
}
