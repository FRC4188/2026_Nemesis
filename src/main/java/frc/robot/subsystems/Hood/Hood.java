package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;

  private final Alert hoodDisconnectedAlert;

  public Hood(HoodIO io) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();

    hoodDisconnectedAlert = new Alert("Hood disconnected", AlertType.kError);
  }

  public Command runVolts(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.runVolts(volts.getAsDouble());
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.runVolts(0);
        },
        this);
  }

  // put a conversion into this if needed
  @AutoLogOutput(key = "Hood/Angle Radians")
  public double getAngleRad() {
    return inputs.angleRads;
  }

  //   @AutoLogOutput(key = "Hood/At Angle")
  //   public boolean atAngle() {
  //     return (Math.abs(getAngleRad() - io.getSetpoint())) <= Constants.HoodConstants.kTolerance;
  //   }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    hoodDisconnectedAlert.set(!inputs.connected);
  }
}
