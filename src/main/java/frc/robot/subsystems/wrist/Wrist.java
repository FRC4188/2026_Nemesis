package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private static Wrist instance = null;

  public static synchronized Wrist getInstance() {
    if (instance == null)
      instance =
          new Wrist(
              switch (Constants.Robot.currentMode) {
                case REAL -> new WristIOReal();
                case SIM -> new WristIOSim();
                case REPLAY -> new WristIO() {};
              });
    return instance;
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private final Alert wristDisconnectedAlert;
  private final StateMachine<WristState> stateMachine =
      new StateMachine<>("Wrist/StateMachine", WristState.IDLE);

  // @AutoLogOutput(key = "Wrist/Setpoint Degrees")
  private double setpoint = 0.0;

  @AutoLogOutput(key = "Wrist/Shake Enable?")
  public boolean shakeEnable = true;

  public void enableShake(boolean enable) {
    shakeEnable = enable;
  }

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();
    wristDisconnectedAlert = new Alert("Wrist motor disconnected.", AlertType.kError);
  }

  public void runWristVolts(double volts) {
    setState(WristState.MANUAL, "runWristVolts");
    setpoint = 180;
    io.setVolts(volts);
  }

  public void stop() {
    setState(WristState.IDLE, "stop");
    io.setVolts(0.0);
  }

  public void stow() {
    setState(WristState.POSITION, "stow");
    setpoint = 144;
    io.setPosition(Constants.WristConstants.Max_A);
  }

  public void down() {
    setState(WristState.POSITION, "down");
    setpoint = 0;
    io.setPosition(Constants.WristConstants.Min_A);
  }

  public void setAngle(double set) {
    setState(WristState.POSITION, "setAngle");
    setpoint = set;
    io.setPosition(Rotation2d.fromDegrees(set));
  }

  public void zero() {
    io.setZero();
  }

  public void runStateVolts(WristState state, double volts) {
    setState(state, "state request");
    setpoint = 180;
    io.setVolts(volts);
  }

  public WristState getState() {
    return stateMachine.getCurrentState();
  }

  private void setState(WristState state, String reason) {
    stateMachine.requestState(state);
    stateMachine.transitionTo(state, reason);
  }

  @AutoLogOutput(key = "Wrist/Angle Degrees")
  public double getAngle() {
    return inputs.position.getDegrees();
  }

  // @AutoLogOutput(key = "Wrist/At Goal?")
  public boolean atGoal() {
    return Math.abs(inputs.position.getDegrees() - setpoint)
        < Constants.WristConstants.kTolerance.getDegrees();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    stateMachine.periodic();

    wristDisconnectedAlert.set(!inputs.connected);
  }
}
