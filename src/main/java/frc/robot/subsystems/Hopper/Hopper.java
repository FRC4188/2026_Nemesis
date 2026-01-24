package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs;

  public Hopper(HopperIO io) {
    this.io = io;
    inputs = new HopperIOInputsAutoLogged();
  }

  public Command load(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVolts(volts.getAsDouble());
        },
        this);
  }

  public Command unload(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVolts(-volts.getAsDouble());
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.runVolts(0);
        },
        this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }
}
