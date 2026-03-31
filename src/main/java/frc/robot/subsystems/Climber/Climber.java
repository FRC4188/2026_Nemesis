package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private static Climber instance = null;
  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber(
      switch(Constants.Robot.currentMode) {
      case REAL -> new ClimberIOReal();
      case SIM -> new ClimberIOSim();
      case REPLAY -> new ClimberIO() {};
    });
    return instance;
  }
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private final Alert climberDisconnectedAlert;

  // @AutoLogOutput(key = "Climber/Setpoint Inches")
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
    io.setPosition(Constants.ClimberConstants.Max_H * Constants.ClimberConstants.kConversion);
  }

  public void lower() {
    setpoint = 0.0;
    io.setPosition(Constants.ClimberConstants.Min_H * Constants.ClimberConstants.kConversion);
  }

  // @AutoLogOutput(key = "Climber/At Goal?")
  public boolean atGoal() {
    return Math.abs(getHeight() - setpoint)
        < Units.metersToInches(Constants.ClimberConstants.kTolerance);
  }

  // @AutoLogOutput(key = "Climber/Is Stalled?")
  public boolean isStalled() {
    return Math.abs(inputs.currentAmps) > Constants.ClimberConstants.kStallCurrent
        && inputs.velRots < 0.1;
  }

  //@AutoLogOutput(key = "Climber/Height Inches")
  public double getHeight() {
    return Units.metersToInches(inputs.posRots / Constants.ClimberConstants.kConversion);
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
