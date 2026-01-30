package frc.robot.subsystems.Loader.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Intake {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final Alert intakeDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    intakeDisconnectedAlert = new Alert("Intake motor disconnected.", AlertType.kError);
  }

  public void setVelocity(double rpm) {
    io.setVelocity(rpm / 60);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void stop() {
    io.runVolts(0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.connected);
  }
}
