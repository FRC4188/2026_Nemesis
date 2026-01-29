package frc.robot.subsystems.Loader.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private double relative_zero = 0;
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
    angle = angle.fromRadians(MathUtil.clamp(angle.getRadians(), 0, Math.PI / 2));

    Logger.recordOutput("Wrist/setPoint", angle);
    io.setPosition(angle);
  }

  @AutoLogOutput(key = "Wrist/Angle Radians")
  public double getAngle() {
    return io.getAngle() - relative_zero;
  }

  public void zero() {
    relative_zero = io.getAngle();
  }

  public void stop() {
    io.runVolts(0);
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }

  public boolean atGoal(double target) {
    return Math.abs(getAngle() - target) < Constants.WristConstants.kTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    wristDisconnectedAlert.set(!inputs.connected);
  }
}
