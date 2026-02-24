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

  public void runShooter(double RPM) {
    shooter.setVelocity(RPM);
  }

  public void setHood(Rotation2d radians) {
    hood.setPosition(radians);
  }

  public void runShooterVolts(double output) {
    shooter.runVolts(output);
  }

  public void runHoodVolts(double output) {
    hood.runVolts(output + Constants.HoodConstants.hoodGains.kG);
  }

  public void updateHoodPID(double kp, double ki, double kd, double kg) {
    hood.updatePID(kp, ki, kd, kg);
  }

  public void updateLShooterPID(double kp, double ki, double kd, double kv) {
    shooter.updateLeftPID(kp, ki, kd, kv);
  }

  public void updateRShooterPID(double kp, double ki, double kd, double kv) {
    shooter.updateLeftPID(kp, ki, kd, kv);
  }

  public boolean hoodAtTarget() {
    return hood.atGoal();
  }

  public boolean shooterAtTarget() {
    return shooter.atGoal();
  }

  public double getHoodAngle() {
    return hood.getAngleRad();
  }

  @Override
  public void periodic() {
    shooter.periodic();
    hood.periodic();
  }
}
