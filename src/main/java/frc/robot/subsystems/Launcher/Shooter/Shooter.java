package frc.robot.subsystems.Launcher.Shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLogOutput;
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

  @AutoLogOutput(key = "Shooter/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(getVelocity() - io.getSetpoint()) < Constants.ShooterConstants.kTolerance;
  }

  public void setVelocity(double RPM) {
    io.setVelocity(RPM);
  }

  public void stop() {
    io.setVelocity(0.0);
  }

  @AutoLogOutput(key = "Shooter/Velocity RPM")
  public double getVelocity() {
    return (inputs.leftVelocityRPM + inputs.rightVelocityRPM) / 2.0;
  }

  public void updatePID(double kp, double ki, double kd, double kf) {
    io.updatePID(kp, ki, kd, kf);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    leftDisconnectedAlert.set(!inputs.leftConnected);
    rightDisconnectedAlert.set(!inputs.rightConnected);
  }
}
