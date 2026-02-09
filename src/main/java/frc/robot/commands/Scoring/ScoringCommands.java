package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher.Launcher;

public class ScoringCommands {

  // redundant example
  public Command WindUp(Launcher launcher, double RPM) {
    return Commands.runOnce(() -> launcher.runShooter(RPM), launcher);
  }
}
