package frc.robot.commands.Scoring;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher.Launcher;

public class ScoringCommands {

  // redundant example
  public static Command WindUp(Launcher launcher, double RPM) {
    return Commands.runOnce(() -> launcher.runShooter(RPM), launcher);
  }

  public Command angleHood(Launcher launcher, Rotation2d angle) {
    return Commands.runOnce(() -> launcher.setHood(angle), launcher);
  }

  //  public Command angleHoodStaticShot(Launcher launcher, double RPM, Translation2d goal){
  //     return Commands.runOnce(() -> launcher.setHood(staticShot(RPM * 18.9839545, goal)),
  //  launcher);
  //    }

  // public Command angleHoodMovingShot(Launcher launcher, double RPM, Translation3d goal,
  // Translation2d robotVel){
  //   return Commands.runOnce(() -> launcher.setHoot(movingShot(RPM * 18.9839545, goal, robotVel)),
  // launcher);
  // }

}
