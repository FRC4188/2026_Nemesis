package frc.robot.subsystems.Transfer.Indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Indexer {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;
  private final Alert indexerDisconnectedAlert;

  public Indexer(IndexerIO io) {
    this.io = io;
    inputs = new IndexerIOInputsAutoLogged();
    indexerDisconnectedAlert = new Alert("Indexer motor disconnected.", AlertType.kError);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void stop() {
    io.runVolts(0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    indexerDisconnectedAlert.set(!inputs.connected);
  }
}
