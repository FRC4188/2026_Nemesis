package frc.robot.subsystems.climber;

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

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private final Alert climberDisconnectedAlert;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    climberDisconnectedAlert = new Alert("Climber motor disconnected.", AlertType.kError);
  }

  public Command runVolts(DoubleSupplier input) {
    return Commands.run(() -> io.runVolts(6 * input.getAsDouble()), this);
  }

  public Command raise() {
    return Commands.runOnce(
        () -> io.setPosition(Rotation2d.fromRotations(Constants.ClimberConstants.Max_R)), this);
  }

  public Command lower() {
    return Commands.runOnce(
        () -> io.setPosition(Rotation2d.fromRotations(Constants.ClimberConstants.Min_R)), this);
  }

  @AutoLogOutput(key = "Climber/At Goal?")
  public boolean atGoal() {
    return Math.abs(inputs.posRots - io.getSetpoint()) < Constants.ClimberConstants.kTolerance;
  }

  @AutoLogOutput(key = "Climber/Height Rotations")
  public double getHeightRots() {
    return inputs.posRots;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    climberDisconnectedAlert.set(!inputs.connected);
  }
}
