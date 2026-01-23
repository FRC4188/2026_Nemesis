package frc.robot.subsystems.Superstructure.Wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private double relative_zero = 0;

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  @AutoLogOutput(key = "Wrist/Angle Radians")
  public double getAngle() {
    return io.getAngle() - relative_zero;
  }

  public void zero() {
    relative_zero = io.getAngle();
  }

  public boolean atGoal(double target) {
    return Math.abs(getAngle() - target) < Constants.WristConstants.kTolerance;
  }
}
