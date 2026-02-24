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

  @AutoLogOutput(key = "Climber/Height Meters")
  public double getHeight() {
    return inputs.posRots * Constants.ClimberConstants.kConverter;
  }

  public void setHeight(double height) {
    io.setPosition(Rotation2d.fromRotations(height / Constants.ClimberConstants.kConverter));
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
