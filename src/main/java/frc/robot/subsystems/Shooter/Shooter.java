package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
  }

  // THIS WILL CHANGE LATER, THIS IS JUST RUNNING VOLTS
  public Command shootleft(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVoltsLeft(volts.getAsDouble());
        },
        this);
  }

  public Command shootRight(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVoltsRight(volts.getAsDouble());
        },
        this);
  }

  public Command stopLeft() {
    return Commands.run(
        () -> {
          io.runVoltsLeft(0);
        },
        this);
  }

  public Command stopRight() {
    return Commands.run(
        () -> {
          io.runVoltsRight(0);
        },
        this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
