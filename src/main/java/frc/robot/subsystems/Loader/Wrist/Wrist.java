package frc.robot.subsystems.Loader.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private final Alert wristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();
    wristDisconnectedAlert = new Alert("Wrist motor disconnected.", AlertType.kError);
  }

  public void runVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    io.runVolts(volts);
  }

  public void setPosition(Rotation2d angle) {
    angle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                Constants.WristConstants.Min_A,
                Constants.WristConstants.Max_A));

    io.setPosition(angle);
  }

  public void reloadPosition() {
    if (!io.isStalled() || Math.abs(inputs.appliedVolts) < 1.0) return;

    io.setPosition(
        Rotation2d.fromRadians(
            inputs.appliedVolts > 0.0
                ? Constants.WristConstants.Max_A
                : Constants.WristConstants.Min_A));
  }

  @AutoLogOutput(key = "Wrist/Angle Radians")
  public double getAngle() {
    return inputs.posRads;
  }

  public void stop() {
    io.runVolts(0);
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }

  @AutoLogOutput(key = "Wrist/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(inputs.posRads - io.getSetpoint()) < Constants.WristConstants.kTolerance;
  }

  public void periodic() {
    // TODO: Needs Testing
    // reloadPosition();

    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    wristDisconnectedAlert.set(!inputs.connected);
  }
}
