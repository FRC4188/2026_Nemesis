package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private final Alert wristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();
    wristDisconnectedAlert = new Alert("Wrist motor disconnected.", AlertType.kError);
  }

  public Command runWrist(DoubleSupplier inputs) {
    return Commands.run(() -> io.runVolts(3 * inputs.getAsDouble()));
  }

  public Command stow() {
    return Commands.runOnce(
        () -> io.setPosition(Rotation2d.fromRadians(Constants.WristConstants.Max_A)), this);
  }

  public Command down() {
    return Commands.runOnce(
        () -> io.setPosition(Rotation2d.fromRadians(Constants.WristConstants.Min_A)), this);
  }

  /**
   * 
   * @return Wrist position in radians
   */
  @AutoLogOutput(key = "Wrist/Angle Radians")
  public double getAngle() {
    return inputs.posRads;
  }

  @AutoLogOutput(key = "Wrist/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(inputs.posRads - io.getSetpoint()) < Constants.WristConstants.kTolerance;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    wristDisconnectedAlert.set(!inputs.connected);
  }
}
