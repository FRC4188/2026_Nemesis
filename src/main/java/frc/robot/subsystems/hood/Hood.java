package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private final Alert hoodDisconnectedAlert;

  @AutoLogOutput(key = "Hood/Setpoint Degrees")
  private double setpoint = 0.0;

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
  }

  public void runHoodVolts(double volts) {
    setpoint = 0.0;
    io.setVolts(MathUtil.clamp(volts, -12, 12));
  }

  public void stop() {
    io.setVolts(0.0);
  }

  public void setAngle(Rotation2d angle) {
    setpoint = angle.getDegrees();
    io.setPosition(
        Rotation2d.fromDegrees(
            MathUtil.clamp(
                angle.getDegrees(),
                Constants.HoodConstants.Min_A.getDegrees(),
                Constants.HoodConstants.Max_A.getDegrees())));
  }

  public void stow() {
    setpoint = 0.0;
    io.setPosition(Rotation2d.kZero);
  }

  public void zero() {
    io.setZero();
  }

  @AutoLogOutput(key = "Hood/Is Stalled?")
  public boolean isStalled() {
    return Math.abs(inputs.currentAmps) > Constants.HoodConstants.kStallCurrent
        && inputs.velocity.getDegrees() < 2.0;
  }

  @AutoLogOutput(key = "Hood/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(inputs.position.getDegrees() - setpoint)
        < Constants.HoodConstants.kTolerance.getDegrees();
  }

  @AutoLogOutput(key = "Hood/Angle Degrees")
  public double getAngle() {
    return inputs.position.getDegrees();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.motorConnected);
  }
}
