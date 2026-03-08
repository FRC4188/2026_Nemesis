package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
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
    return Math.abs(getAverageVelocity() - io.getSetpoint())
        < Constants.ShooterConstants.kTolerance;
  }

  public Command setVelocity(DoubleSupplier RPM) {
    return Commands.runEnd(() -> io.setVelocity(RPM.getAsDouble()), () -> io.runVolts(0.0), this);
  }

  public Command setVelocity(double RPM) {
    return Commands.runOnce(() -> io.setVelocity(RPM), this);
  }

  public Command runVolts(DoubleSupplier input) {
    return Commands.runEnd(
        () -> io.runVolts(12 * input.getAsDouble()), () -> io.runVolts(0.0), this);
  }

  public Command runVolts(double voltage) {
    return Commands.runOnce(() -> io.runVolts(voltage), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> io.runVolts(0.0), this);
  }

  @AutoLogOutput(key = "Shooter/Average Velocity RPM")
  public double getAverageVelocity() {
    return (inputs.rightVelocityRPM + inputs.leftVelocityRPM) / 2.0;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    leftDisconnectedAlert.set(!inputs.leftConnected);
    rightDisconnectedAlert.set(!inputs.rightConnected);
  }
}
