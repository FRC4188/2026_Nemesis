package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private static Hopper instance = null;

  public static synchronized Hopper getInstance() {
    if (instance == null)
      instance =
          new Hopper(
              switch (Constants.Robot.currentMode) {
                case REAL -> new HopperIOReal();
                case SIM -> new HopperIOSim();
                case REPLAY -> new HopperIO() {};
              });
    return instance;
  }

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

  public void runHopperVolts(double a_volts, double i_volts) {
    io.setAggitateVolts(a_volts);
    io.setIndexerVolts(i_volts);
  }

  public void stop() {
    io.setAggitateVolts(0.0);
    io.setIndexerVolts(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    aggitateDisconnectedAlert.set(!inputs.aggitateConnected);
    indexerDisconnectedAlert.set(!inputs.indexerConnected);
  }
}
