package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final Alert intakeDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    intakeDisconnectedAlert = new Alert("Intake motor disconnected.", AlertType.kError);
  }

  public Command intake(DoubleSupplier inputs) {
    return Commands.run(() -> io.runVolts(12 * inputs.getAsDouble()), this);
  }

  public Command intake(double voltage) {
    return Commands.runOnce(() -> io.runVolts(voltage), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> io.runVolts(0.0), this);
  }

  @AutoLogOutput(key = "Intake/Is Stalled?")
  public boolean isStalled() {
    return io.isStalled();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.connected);
  }
}
