package frc.robot.subsystems.Transfer.Hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Hopper {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs;
  private final Alert hopperDisconnectedAlert;

  public Hopper(HopperIO io) {
    this.io = io;
    inputs = new HopperIOInputsAutoLogged();
    hopperDisconnectedAlert = new Alert("Hopper motor disconnected.", AlertType.kError);
  }

  public void runVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    io.runVolts(volts);
  }

  public void stop() {
    io.runVolts(0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    hopperDisconnectedAlert.set(!inputs.connected);
  }
}
