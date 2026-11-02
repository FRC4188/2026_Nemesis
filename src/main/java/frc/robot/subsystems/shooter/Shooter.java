package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum State {
    IDLE,
    VELOCITY,
    TORQUE_CURRENT
  }

  private static Shooter instance = null;

  public static synchronized Shooter getInstance() {
    if (instance == null)
      instance =
          new Shooter(
              switch (Constants.Robot.currentMode) {
                case REAL -> new ShooterIOReal();
                case SIM -> new ShooterIOSim();
                case REPLAY -> new ShooterIO() {};
              });
    return instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final Alert leftDisconnectedAlert;
  private final Alert rightDisconnectedAlert;
  private final Alert left2DisconnectedAlert;
  private final Alert left3DisconnectedAlert;

  @AutoLogOutput(key = "Shooter/Setpoint RPM")
  private double setRPM = 0.0;

  private double wantedTorqueCurrent;
  private final StateMachine<State> stateMachine =
      new StateMachine<>("Shooter", State.IDLE) {
        @Override
        protected void runStateMachine() {
          if (getCurrentState() != getWantedState()) {
            transitionTo(getWantedState(), "Requested");
          }
          switch (getCurrentState()) {
            case IDLE -> io.setTorqueCurrent(0.0);
            case VELOCITY -> io.setVelocity(setRPM);
            case TORQUE_CURRENT -> io.setTorqueCurrent(wantedTorqueCurrent);
          }
        }
      };

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    leftDisconnectedAlert = new Alert("Left shooter motor disconnected.", AlertType.kError);
    rightDisconnectedAlert = new Alert("Right shooter motor disconnected.", AlertType.kError);
    left2DisconnectedAlert = new Alert("Left 2 shooter motor disconnected.", AlertType.kError);
    left3DisconnectedAlert = new Alert("Left 3 shooter motor disconnected.", AlertType.kError);
  }

  public void setVelocityRPM(double RPM) {
    setRPM = MathUtil.clamp(RPM, 0, Constants.ShooterConstants.kMaxRPM);
    stateMachine.requestState(State.VELOCITY);
  }

  public void runTC(double amps) {
    setRPM = 0;
    wantedTorqueCurrent = amps;
    stateMachine.requestState(State.TORQUE_CURRENT);
  }

  public void stop() {
    setRPM = 0;
    stateMachine.requestState(State.IDLE);
  }

  public boolean atGoal() {
    return Math.abs(setRPM - ((inputs.leftVelocityRPM + inputs.rightVelocityRPM) / 2.0))
        < Constants.ShooterConstants.kTolerance;
  }

  @AutoLogOutput(key = "Shooter/Average Acceleration")
  public double getAverageAcceleration() {
    return io.getAcceleration();
  }

  public double getSpeeds() {
    return io.getSpeeds();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    stateMachine.periodic();

    leftDisconnectedAlert.set(!inputs.leftConnected);
    rightDisconnectedAlert.set(!inputs.rightConnected);
    left2DisconnectedAlert.set(!inputs.left2Connected);
    left3DisconnectedAlert.set(!inputs.left3Connected);
  }

  public State getWantedState() {
    return stateMachine.getWantedState();
  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }
}
