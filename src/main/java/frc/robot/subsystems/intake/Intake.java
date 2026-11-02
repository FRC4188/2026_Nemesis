package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.statemachine.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum State {
    IDLE,
    INTAKING,
    EJECTING
  }

  private static Intake instance = null;

  public static synchronized Intake getInstance() {
    if (instance == null)
      instance =
          new Intake(
              switch (Constants.Robot.currentMode) {
                case REAL -> new IntakeIOReal();
                case SIM -> new IntakeIOSim();
                case REPLAY -> new IntakeIO() {};
              });
    return instance;
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final Alert intakeDisconnectedAlert;
  private double wantedVolts;
  private final StateMachine<State> stateMachine =
      new StateMachine<>("Intake", State.IDLE) {
        @Override
        protected void runStateMachine() {
          if (getCurrentState() != getWantedState()) {
            transitionTo(getWantedState(), "Requested");
          }
          switch (getCurrentState()) {
            case IDLE -> io.setVolts(0.0);
            case INTAKING, EJECTING -> io.setVolts(wantedVolts);
          }
        }
      };

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    intakeDisconnectedAlert = new Alert("Intake motor disconnected.", AlertType.kError);
  }

  public void intakeVolts(double volts) {
    wantedVolts = volts;
    stateMachine.requestState(
        volts == 0.0 ? State.IDLE : volts < 0.0 ? State.EJECTING : State.INTAKING);
  }

  public void ejectVolts(double volts) {
    wantedVolts = -Math.abs(volts);
    stateMachine.requestState(State.EJECTING);
  }

  public void stop() {
    wantedVolts = 0.0;
    stateMachine.requestState(State.IDLE);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    stateMachine.periodic();

    intakeDisconnectedAlert.set(!inputs.connected);
  }

  public State getWantedState() {
    return stateMachine.getWantedState();
  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }
}
