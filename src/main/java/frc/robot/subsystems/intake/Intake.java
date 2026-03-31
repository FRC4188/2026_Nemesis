package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private static Intake instance = null;

  public static synchronized Intake getInstance() {
    if (instance == null)
      instance =
          new Intake(
              switch (Constants.Robot.currentMode) {
                case REAL -> new IntakeIOReal();
                case SIM -> new IntakeIOSim();
                case REPLAY -> new IntakeIO() {};
              });
    return instance;
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final Alert intakeDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    intakeDisconnectedAlert = new Alert("Intake motor disconnected.", AlertType.kError);
  }

  public void intakeVolts(double volts) {
    io.setVolts(MathUtil.clamp(volts, -12, 12));
  }

  public void ejectVolts(double volts) {
    intakeVolts(-volts);
  }

  public void stop() {
    io.setVolts(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.connected);
  }
}
