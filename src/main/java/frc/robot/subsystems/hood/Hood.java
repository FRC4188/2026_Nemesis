package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {
  private static Hood instance = null;

  public static synchronized Hood getInstance() {
    if (instance == null)
      instance =
          new Hood(
              switch (Constants.Robot.currentMode) {
                case REAL -> new HoodIOReal();
                case SIM -> new HoodIOSim();
                case REPLAY -> new HoodIO() {};
              });
    return instance;
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private final Alert hoodDisconnectedAlert;

  @AutoLogOutput(key = "Hood/Setpoint Degrees")
  private double setpoint = 0.0;

  private LoggedNetworkNumber offset = new LoggedNetworkNumber("Hood/Offset Degrees Incline", 8.0);

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
  }

  public void runHoodVolts(double volts) {
    setpoint = 0.0;
    io.setVolts(volts);
  }

  public void stop() {
    io.setVolts(0.0);
  }

  public void addOne() {
    offset.set(offset.get() + 1);
  }

  public void subOne() {
    offset.set(offset.get() - 1);
  }

  public void setAngle(Rotation2d angle) {
    setpoint = angle.getDegrees() + offset.getAsDouble();
    io.setPosition(
        Rotation2d.fromDegrees(
            MathUtil.clamp(
                setpoint,
                Constants.HoodConstants.Min_A.getDegrees(),
                Constants.HoodConstants.Max_A.getDegrees())));
  }

  public void stow() {
    setpoint = 0.0;
    setAngle(Rotation2d.kZero);
  }

  public void zero() {
    io.setZero();
  }

  @AutoLogOutput(key = "Hood/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(getAngle() - setpoint) < Constants.HoodConstants.kTolerance.getDegrees();
  }

  @AutoLogOutput(key = "Hood/Angle Degrees")
  public double getAngle() {
    return inputs.position.getDegrees() - offset.getAsDouble();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.motorConnected);
  }
}
