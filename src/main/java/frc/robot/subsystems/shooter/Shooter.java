package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  private final Alert leftDisconnectedAlert;
  private final Alert rightDisconnectedAlert;

  // @AutoLogOutput(key = "Shooter/Right Setpoint RPM")
  private double setRightRPM = 0.0;

  // @AutoLogOutput(key = "Shooter/Left Setpoint RPM")
  private double setLeftRPM = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();

    leftDisconnectedAlert = new Alert("Left shooter motor disconnected.", AlertType.kError);
    rightDisconnectedAlert = new Alert("Right shooter motor disconnected.", AlertType.kError);
  }

  public void setVelocityRPM(double leftRPM, double rightRPM) {
    setRightRPM = rightRPM;
    setLeftRPM = leftRPM;
    io.setVelocity(
        MathUtil.clamp(leftRPM, 0, Constants.ShooterConstants.kMaxRPM),
        MathUtil.clamp(rightRPM, 0, Constants.ShooterConstants.kMaxRPM));
  }

  public void stop() {
    setRightRPM = 0;
    setLeftRPM = 0;
    io.setVolts(0.0);
  }

  // @AutoLogOutput(key = "Shooter/At Goal?")
  public boolean atGoal() {
    return leftAtGoal() && rightAtGoal();
  }

  public boolean leftAtGoal() {
    return Math.abs(inputs.leftVelocityRPM - setLeftRPM) < Constants.ShooterConstants.kTolerance;
  }

  public boolean rightAtGoal() {
    return Math.abs(inputs.rightVelocityRPM - setRightRPM) < Constants.ShooterConstants.kTolerance;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    leftDisconnectedAlert.set(!inputs.leftConnected);
    rightDisconnectedAlert.set(!inputs.rightConnected);
  }
}
