package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private final Alert hoodDisconnectedAlert;

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood motor disconnected.", AlertType.kError);
  }

  public Command runVolts(DoubleSupplier input) {
    return Commands.runEnd(
        () -> io.runVolts(3 * input.getAsDouble()), () -> io.runVolts(0.0), this);
  }

  public Command setPosition(Supplier<Rotation2d> angle) {
    return Commands.runEnd(
        () ->
            io.setPosition(
                Rotation2d.fromRadians(
                    Math.PI / 2.0
                        - MathUtil.clamp(
                            angle.get().getRadians(),
                            Constants.HoodConstants.Min_A,
                            Constants.HoodConstants.Max_A))),
        () -> io.setPosition(Rotation2d.fromDegrees(10.0)),
        this);
  }

  public Command zero() {
    return Commands.runOnce(() -> io.zero());
  }

  public Command setPosition(Rotation2d angle) {
    return Commands.runOnce(
        () ->
            io.setPosition(
                Rotation2d.fromRadians(
                    Math.PI / 2.0
                        - MathUtil.clamp(
                            angle.getRadians(),
                            Constants.HoodConstants.Min_A,
                            Constants.HoodConstants.Max_A))),
        this);
  }

  public Command stow() {
    return Commands.runOnce(() -> io.setPosition(Rotation2d.kZero), this);
  }

  @AutoLogOutput(key = "Hood/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(inputs.posRads - io.getSetpoint()) < Constants.HoodConstants.kTolerance;
  }

  /**
   * @return Hood angle in radians
   */
  @AutoLogOutput(key = "Hood/Shooting Angle Degrees")
  public double getShotAngle() {
    return Units.radiansToDegrees((Math.PI / 2.0 - inputs.posRads));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.motorConnected);
  }
}
