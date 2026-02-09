package frc.robot.subsystems.Launcher.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private final Alert hoodDisconnectedAlert;
  private final Alert canDisconnectedAlert;

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
    canDisconnectedAlert = new Alert("Hood CANcoder disconnected.", AlertType.kError);
  }

  public void runVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);

    io.setOpenLoop(volts);
  }

  public void setPosition(Rotation2d angle) {
    angle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(), Constants.HoodConstants.Min_A, Constants.HoodConstants.Max_A));
    io.setPosition(angle);
  }

  public boolean atGoal() {
    return Math.abs(getAngleRad() - io.getSetpoint()) < Constants.HoodConstants.kTolerance;
  }

  @AutoLogOutput(key = "Hood/Angle Radians")
  public double getAngleRad() {
    return inputs.posRads;
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.motorConnected);
    canDisconnectedAlert.set(!inputs.coderConnected);
  }
}
