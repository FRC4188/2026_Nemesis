package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final Alert leftDisconnectedAlert;
  private final Alert rightDisconnectedAlert;

  @AutoLogOutput(key = "Shooter/Setpoint RPM")
  private double setRPM = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    leftDisconnectedAlert = new Alert("Left shooter motor disconnected.", AlertType.kError);
    rightDisconnectedAlert = new Alert("Right shooter motor disconnected.", AlertType.kError);
  }

  public void setVelocityRPM(double RPM) {
    setRPM = RPM;
    io.setVelocity(MathUtil.clamp(RPM, 0, Constants.ShooterConstants.kMaxRPM));
  }

  public void runShooterVolts(double volts) {
    setRPM = -100;
    io.setVolts(MathUtil.clamp(volts, -12, 12));
  }

  public void stop() {
    io.setVolts(0.0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    leftDisconnectedAlert.set(!inputs.leftConnected);
    rightDisconnectedAlert.set(!inputs.rightConnected);
  }
}
