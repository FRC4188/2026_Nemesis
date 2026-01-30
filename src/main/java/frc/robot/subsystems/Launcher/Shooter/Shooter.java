package frc.robot.subsystems.Launcher.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Shooter {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final Alert leftDisconnectedAlert;
  private final Alert rightDisconnectedAlert;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    leftDisconnectedAlert = new Alert("Left shooter motor disconnected.", AlertType.kError);
    rightDisconnectedAlert = new Alert("Right shooter motor disconnected.", AlertType.kError);
  }

  // THIS WILL CHANGE LATER, THIS IS JUST RUNNING VOLTS
  // ALSO NEED TO ADD SET VELOCITY AND TORQUE PID
  public void runVoltsLeft(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    io.runVoltsLeft(volts);
  }

  public void runVoltsRight(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    io.runVoltsRight(volts);
  }

  public void stopRight() {
    io.runVoltsRight(0);
  }

  public void stopLeft() {
    io.runVoltsLeft(0);
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    leftDisconnectedAlert.set(!inputs.left_connected);
    rightDisconnectedAlert.set(!inputs.right_connected);
  }
}
