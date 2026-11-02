package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {
  public enum State {
    IDLE,
    POSITION,
    VOLTAGE,
    ZEROING
  }

  private static Hood instance = null;

  public static synchronized Hood getInstance() {
    if (instance == null)
      instance =
          new Hood(
              switch (Constants.Robot.currentMode) {
                case REAL -> new HoodIOReal();
                case SIM -> new HoodIOSim();
                case REPLAY -> new HoodIO() {};
              });
    return instance;
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private final Alert hoodDisconnectedAlert;

  @AutoLogOutput(key = "Hood/Setpoint Degrees")
  private double setpoint = 0.0;

  private LoggedNetworkNumber offset = new LoggedNetworkNumber("Hood/Offset Degrees Incline", 0.0);
  private double wantedVoltage;
  private final StateMachine<State> stateMachine =
      new StateMachine<>("Hood", State.IDLE) {
        @Override
        protected void runStateMachine() {
          if (getCurrentState() != getWantedState()) {
            transitionTo(getWantedState(), "Requested");
          }
          switch (getCurrentState()) {
            case IDLE -> io.setVolts(0.0);
            case POSITION -> io.setPosition(Rotation2d.fromDegrees(setpoint));
            case VOLTAGE -> io.setVolts(wantedVoltage);
            case ZEROING -> {
              io.setZero();
              requestState(State.IDLE);
            }
          }
        }
      };

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
  }

  public void runHoodVolts(double volts) {
    wantedVoltage = volts;
    setpoint = 0.0;
    stateMachine.requestState(State.VOLTAGE);
  }

  public void stop() {
    stateMachine.requestState(State.IDLE);
  }

  public void addOne() {
    offset.set(offset.get() + 1);
  }

  public void subOne() {
    offset.set(offset.get() - 1);
  }

  public void setAngle(Rotation2d angle) {
    setpoint =
        MathUtil.clamp(
            angle.getDegrees() + offset.getAsDouble(),
            Constants.HoodConstants.Min_A.getDegrees(),
            Constants.HoodConstants.Max_A.getDegrees());
    stateMachine.requestState(State.POSITION);
  }

  public void stow() {
    setpoint = 0.0;
    setAngle(Rotation2d.kZero);
  }

  public void zero() {
    stateMachine.requestState(State.ZEROING);
  }

  @AutoLogOutput(key = "Hood/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(getAngle() - setpoint) < Constants.HoodConstants.kTolerance.getDegrees();
  }

  @AutoLogOutput(key = "Hood/Angle Degrees")
  public double getAngle() {
    return inputs.position.getDegrees() + offset.getAsDouble(); // does this work
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    stateMachine.periodic();

    hoodDisconnectedAlert.set(!inputs.motorConnected);
  }

  public State getWantedState() {
    return stateMachine.getWantedState();
  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }
}
