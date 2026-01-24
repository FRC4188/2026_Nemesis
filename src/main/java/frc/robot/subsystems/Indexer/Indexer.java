package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;
  private final Timer timer;

  public Indexer(IndexerIO io) {
    this.io = io;
    inputs = new IndexerIOInputsAutoLogged();
    timer = new Timer();
  }

  public Command spin(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVolts(volts.getAsDouble());
        },
        this);
  }

  public Command spinReverse(DoubleSupplier volts) {
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

  @AutoLogOutput(key = "Indexer/Is Stalled?")
  public boolean isStalled() {
    return io.isStalled();
  }

  @Override
  public void periodic() {
    io.UpdateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }
}
