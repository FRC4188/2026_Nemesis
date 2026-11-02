package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum State {
    IDLE,
    RUNNING
  }

  @AutoLogOutput(key = "Indexer/Setpoint RPM")
  private double setRPM = 0.0;

  private double wantedAgitatorVolts;
  private final StateMachine<State> stateMachine =
      new StateMachine<>("Hopper", State.IDLE) {
        @Override
        protected void runStateMachine() {
          if (getCurrentState() != getWantedState()) {
            transitionTo(getWantedState(), "Requested");
          }
          switch (getCurrentState()) {
            case IDLE -> {
              io.setAggitateVolts(0.0);
              io.setIndexerVolts(0.0);
            }
            case RUNNING -> {
              io.setAggitateVolts(wantedAgitatorVolts);
              io.setIndexerVelocity(setRPM);
            }
          }
        }
      };

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

  public void runHopper(double a_volts, double iRPM) {
    wantedAgitatorVolts = a_volts;
    setRPM = iRPM;
    stateMachine.requestState(State.RUNNING);
  }

  public boolean indexAtGoal() {
    return setRPM - inputs.indexerRPM < 50;
  }

  public void stop() {
    setRPM = 0.0;
    wantedAgitatorVolts = 0.0;
    stateMachine.requestState(State.IDLE);
  }

  public double getIndexerSpeed() {
    return inputs.indexerRPM;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    stateMachine.periodic();

    aggitateDisconnectedAlert.set(!inputs.aggitateConnected);
    indexerDisconnectedAlert.set(!inputs.indexerConnected);
  }

  public State getWantedState() {
    return stateMachine.getWantedState();
  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }
}
