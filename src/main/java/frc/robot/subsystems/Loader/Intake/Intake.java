package frc.robot.subsystems.Loader.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
  }

  public Command ingest(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVolts(volts.getAsDouble());
        },
        this);
  }

  public Command eject(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVolts(-volts.getAsDouble());
        },
        this);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.runVolts(0);
        },
        this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
