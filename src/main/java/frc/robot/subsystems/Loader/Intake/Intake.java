package frc.robot.subsystems.Loader.Intake;

import edu.wpi.first.math.MathUtil;
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

  public void runVolts(double volts) {
    volts = MathUtil.clamp(-12, volts, 12);
    io.runVolts(volts);
  }

  public void stop() {
    io.runVolts(0);
  }

  public boolean isStalled() {
    return io.isStalled();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.connected);
  }
}
