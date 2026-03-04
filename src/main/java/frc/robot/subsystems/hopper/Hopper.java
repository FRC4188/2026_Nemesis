package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs;

  private final Alert aggitateDisconnectedAlert;
  private final Alert indexerDisconnectedAlert;

  public Hopper(HopperIO io) {
    this.io = io;
    inputs = new HopperIOInputsAutoLogged();
    aggitateDisconnectedAlert = new Alert("Aggitatation motor disconnected.", AlertType.kError);
    indexerDisconnectedAlert = new Alert("Indexer motor disconnected.", AlertType.kError);
  }

  public Command runVolts(DoubleSupplier a_input, DoubleSupplier i_input) {
    return Commands.runEnd(
        () -> {
          io.runAggitateVolts(12 * a_input.getAsDouble());
          io.runIndexerVolts(12 * i_input.getAsDouble());
        },
        () -> {
          io.runAggitateVolts(0.0);
          io.runIndexerVolts(0.0);
        },
        this);
  }

  public Command runVolts(double a_volts, double i_volts) {
    return Commands.runOnce(
        () -> {
          io.runAggitateVolts(a_volts);
          io.runIndexerVolts(i_volts);
        },
        this);
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          io.runAggitateVolts(0.0);
          io.runIndexerVolts(0.0);
        },
        this);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    aggitateDisconnectedAlert.set(!inputs.aggitateConnected);
    indexerDisconnectedAlert.set(!inputs.indexerConnected);
  }
}
