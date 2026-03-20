package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private final Alert wristDisconnectedAlert;

  @AutoLogOutput(key = "Wrist/Setpoint Degrees")
  private double setpoint = 0.0;

  @AutoLogOutput(key = "Wrist/Shake Enable?")
  public boolean shakeEnable = true;

  public void enableShake(boolean enable) {
    shakeEnable = enable;
  }

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();
    wristDisconnectedAlert = new Alert("Wrist motor disconnected.", AlertType.kError);
  }

  // negative is up, positive is down
  public void runWristVolts(double volts) {
    setpoint = 180;
    io.setVolts(MathUtil.clamp(volts, -12, 12));
  }

  public void runWristTC(double amps) {
    setpoint = 180;
    io.setTorqueCurrent(
        MathUtil.clamp(
            amps,
            -Constants.WristConstants.kPeakReverseTC,
            Constants.WristConstants.kPeakForwardTC));
  }

  public void stop() {
    io.setVolts(0.0);
  }

  public void stow() {
    setpoint = 144;
    io.setPosition(Constants.WristConstants.Max_A);
  }

  public void down() {
    setpoint = 0;
    io.setPosition(Constants.WristConstants.Min_A);
  }

  public void zero() {
    io.setZero();
  }

  @AutoLogOutput(key = "Wrist/is Stalling?")
  public boolean isStalled() {
    return Math.abs(inputs.currentAmps) > Constants.WristConstants.kStallCurrent
        && inputs.velocity.getDegrees() < 2.0;
  }

  @AutoLogOutput(key = "Wrist/Angle Degrees")
  public double getAngle() {
    return inputs.position.getDegrees();
  }

  @AutoLogOutput(key = "Wrist/At Goal?")
  public boolean atGoal() {
    return Math.abs(inputs.position.getDegrees() - setpoint)
        < Constants.WristConstants.kTolerance.getDegrees();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    wristDisconnectedAlert.set(!inputs.connected);
  }
}
