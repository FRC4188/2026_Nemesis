package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher.Hood.Hood;
import frc.robot.subsystems.Launcher.Hood.HoodIO;
import frc.robot.subsystems.Launcher.Shooter.Shooter;
import frc.robot.subsystems.Launcher.Shooter.ShooterIO;

public class Launcher extends SubsystemBase {
  private final Shooter shooter;
  private final Hood hood;

  public Launcher(ShooterIO shooterIO, HoodIO hoodIO) {
    shooter = new Shooter(shooterIO);
    hood = new Hood(hoodIO);
  }

  public void runShooterLeft(double volts) {
    shooter.runVoltsLeft(volts);
  }

  public void runShooterRight(double volts) {
    shooter.runVoltsRight(volts);
  }

  public void setHood(Rotation2d radians) {
    hood.setPosition(radians);
  }

  public boolean atHoodGoal(Rotation2d radians, double tolerance) {
    return Math.abs((hood.getAngleRad() - radians.getRadians())) < tolerance;
  }

  public boolean atHoodGoal(Rotation2d radians) {
    return Math.abs(hood.getAngleRad() - radians.getRadians())
        < Constants.WristConstants.kTolerance;
  }

  public void updateHoodPID(double kp, double ki, double kd, double kg) {
    hood.updatePID(kp, ki, kd, kg);
  }

  public void updateShooterPID(double kp, double ki, double kd, double kg) {
    shooter.updatePID(kp, ki, kd, kg);
  }

  @Override
  public void periodic() {
    shooter.periodic();
    hood.periodic();
  }
}
