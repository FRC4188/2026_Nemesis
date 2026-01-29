package frc.robot.subsystems.Launcher.Hood;

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

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
  }

  public void runVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);

    io.runVolts(volts);
  }

  public void setPosition(Rotation2d angle) {
    angle = angle.fromRadians(MathUtil.clamp(angle.getRadians(), 0, Math.PI / 2));
    Logger.recordOutput("Hood/setPoint", angle);
    io.setPosition(angle);
  }

  public boolean atGoal(double target) {
    return Math.abs(getAngleRad() - target) < Constants.HoodConstants.kTolerance;
  }

  // put a conversion into this if needed
  @AutoLogOutput(key = "Hood/Angle Radians")
  public double getAngleRad() {
    return inputs.angleRads;
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.connected);
  }
}
