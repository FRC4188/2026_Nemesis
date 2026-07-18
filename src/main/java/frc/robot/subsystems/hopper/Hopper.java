package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  @AutoLogOutput(key = "Indexer/Setpoint RPM")
  private double setRPM = 0.0;

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
  private final StateMachine<HopperState> stateMachine =
      new StateMachine<>("Hopper/StateMachine", HopperState.IDLE);

  public Hopper(HopperIO io) {
    this.io = io;
    inputs = new HopperIOInputsAutoLogged();
    aggitateDisconnectedAlert = new Alert("Aggitatation motor disconnected.", AlertType.kError);
    indexerDisconnectedAlert = new Alert("Indexer motor disconnected.", AlertType.kError);
  }

  public void runHopper(double a_volts, double iRPM) {
    HopperState state =
        a_volts < 0.0
            ? HopperState.REVERSE
            : (iRPM != 0.0 ? HopperState.INDEXING : HopperState.FEEDING);
    setState(state, "runHopper");
    io.setAggitateVolts(a_volts);
    io.setIndexerVelocity(iRPM);
  }

  public boolean indexAtGoal() {
    return setRPM - inputs.indexerRPM < 50;
  }

  public void stop() {
    setState(HopperState.IDLE, "stop");
    io.setAggitateVolts(0.0);
    io.setIndexerVolts(0.0);
  }

  public HopperState getState() {
    return stateMachine.getCurrentState();
  }

  private void setState(HopperState state, String reason) {
    stateMachine.requestState(state);
    stateMachine.transitionTo(state, reason);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    stateMachine.periodic();

    aggitateDisconnectedAlert.set(!inputs.aggitateConnected);
    indexerDisconnectedAlert.set(!inputs.indexerConnected);
  }
}
