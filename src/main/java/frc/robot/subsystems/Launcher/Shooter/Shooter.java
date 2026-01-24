package frc.robot.subsystems.Launcher.Shooter;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class Shooter {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
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

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }
}
