package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
    return Commands.run(() -> io.runVolts(3 * input.getAsDouble()), this);
  }

  public Command setPosition(Supplier<Rotation2d> angle) {
    return Commands.run(
        () ->
            io.setPosition(
                Rotation2d.fromRadians(
                    Math.PI / 2.0
                        - MathUtil.clamp(
                            angle.get().getRadians(),
                            Constants.HoodConstants.Min_A,
                            Constants.HoodConstants.Max_A))),
        this);
  }

  @AutoLogOutput(key = "Hood/At Setpoint?")
  public boolean atGoal() {
    return Math.abs(inputs.posRads - io.getSetpoint()) < Constants.HoodConstants.kTolerance;
  }

  @AutoLogOutput(key = "Hood/Shooting Angle Rads")
  public double getShotAngle() {
    return Math.PI / 2.0 - inputs.posRads;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.motorConnected);
  }
}
