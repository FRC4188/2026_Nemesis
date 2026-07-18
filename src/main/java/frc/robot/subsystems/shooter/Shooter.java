package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

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
  private final StateMachine<ShooterState> stateMachine =
      new StateMachine<>("Shooter/StateMachine", ShooterState.IDLE);

  @AutoLogOutput(key = "Shooter/Setpoint RPM")
  private double setRPM = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    leftDisconnectedAlert = new Alert("Left shooter motor disconnected.", AlertType.kError);
    rightDisconnectedAlert = new Alert("Right shooter motor disconnected.", AlertType.kError);
    left2DisconnectedAlert = new Alert("Left 2 shooter motor disconnected.", AlertType.kError);
    left3DisconnectedAlert = new Alert("Left 3 shooter motor disconnected.", AlertType.kError);
  }

  public void setVelocityRPM(double RPM) {
    if (stateMachine.getCurrentState() != ShooterState.SHOOTING) {
      setState(ShooterState.SPINNING_UP, "setVelocityRPM");
    }
    setRPM = MathUtil.clamp(RPM, 0, Constants.ShooterConstants.kMaxRPM);
    io.setVelocity(MathUtil.clamp(RPM, 0, Constants.ShooterConstants.kMaxRPM));
  }

  public void runTC(double amps) {
    setState(ShooterState.TORQUE_CURRENT, "runTC");
    setRPM = 0;
    io.setTorqueCurrent(amps);
  }

  public void stop() {
    setState(ShooterState.IDLE, "stop");
    setRPM = 0;
    io.setTorqueCurrent(0.0);
  }

  public boolean atGoal() {
    return setRPM - ((inputs.leftVelocityRPM + inputs.rightVelocityRPM) / 2.0)
        < Constants.ShooterConstants.kTolerance;
  }

  public ShooterState getState() {
    return stateMachine.getCurrentState();
  }

  public void markShooting() {
    setState(ShooterState.SHOOTING, "feeding");
  }

  private void setState(ShooterState state, String reason) {
    stateMachine.requestState(state);
    stateMachine.transitionTo(state, reason);
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
}
