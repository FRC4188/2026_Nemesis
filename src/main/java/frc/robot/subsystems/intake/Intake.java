package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.statemachine.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

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
  private final StateMachine<IntakeState> stateMachine =
      new StateMachine<>("Intake/StateMachine", IntakeState.IDLE);

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    intakeDisconnectedAlert = new Alert("Intake motor disconnected.", AlertType.kError);
  }

  public void intakeVolts(double volts) {
    IntakeState state =
        Math.abs(volts) <= 1.5
            ? IntakeState.HOLDING_LOW_POWER
            : (Math.abs(volts - 8.75) < 1e-9 ? IntakeState.INTAKING : IntakeState.AUTO_INTAKING);
    setState(state, "intakeVolts");
    io.setVolts(volts);
  }

  public void ejectVolts(double volts) {
    setState(IntakeState.EJECTING, "ejectVolts");
    io.setVolts(-volts);
  }

  public void stop() {
    setState(IntakeState.IDLE, "stop");
    io.setVolts(0.0);
  }

  public IntakeState getState() {
    return stateMachine.getCurrentState();
  }

  private void setState(IntakeState state, String reason) {
    stateMachine.requestState(state);
    stateMachine.transitionTo(state, reason);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    stateMachine.periodic();

    intakeDisconnectedAlert.set(!inputs.connected);
  }
}
