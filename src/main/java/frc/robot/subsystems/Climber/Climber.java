package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    climberDisconnectedAlert = new Alert("Climber motor disconnected.", AlertType.kError);
  }

  public void runVolts(double output) {
    io.runVolts(output);
  }

  @AutoLogOutput(key = "Climber/Height Rotations")
  public double getHeightRots() {
    return inputs.posRots;
  }

  public void fall() {
    io.setPosition(Rotation2d.fromRotations(Constants.ClimberConstants.Max_R), 1);
  }

  public void climb() {
    io.setPosition(Rotation2d.fromRotations(Constants.ClimberConstants.Min_R), 1);
  }

  public void stow() {
    io.setPosition(Rotation2d.fromRotations(Constants.ClimberConstants.Min_R), 0);
  }

  public void ready() {
    io.setPosition(Rotation2d.fromRotations(Constants.ClimberConstants.Max_R), 0);
  }

  public boolean atGoal() {
    return Math.abs(inputs.posRots - io.getSetpoint()) < Constants.ClimberConstants.kTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    climberDisconnectedAlert.set(!inputs.connected);
  }
}
