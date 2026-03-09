package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private final Alert climberDisconnectedAlert;

  @AutoLogOutput(key = "Climber/Height Meters")
  private double setpoint = 0.0;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    climberDisconnectedAlert = new Alert("Climber motor disconnected.", AlertType.kError);
  }

  public void runClimberVolts(double volts) {
    setpoint = -1;
    io.setVolts(MathUtil.clamp(volts, -12, 12));
  }

  public void stop() {
    io.setVolts(0.0);
  }

  public void raise() {
    setpoint = 7.5;
    io.setPosition(Constants.ClimberConstants.Max_H);
  }

  public void lower() {
    setpoint = 0.0;
    io.setPosition(Constants.ClimberConstants.Min_H);
  }

  @AutoLogOutput(key = "Climber/At Goal?")
  public boolean atGoal() {
    return Math.abs(inputs.posMeters - setpoint) < Constants.ClimberConstants.kTolerance;
  }

  @AutoLogOutput(key = "Climber/Is Stalled?")
  public boolean isStalled() {
    return Math.abs(inputs.currentAmps) > Constants.ClimberConstants.kStallCurrent
        && inputs.velMeters < 0.05;
  }

  @AutoLogOutput(key = "Climber/Height Meters")
  public double getHeight() {
    return inputs.posMeters;
  }

  public void zero() {
    io.setZero();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    climberDisconnectedAlert.set(!inputs.connected);
  }
}
