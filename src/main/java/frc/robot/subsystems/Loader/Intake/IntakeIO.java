package frc.robot.subsystems.Loader.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRotPerSec = 0.0;

    public boolean connected = true;
    public double appliedVolts = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void setVelocity(double rotations) {}
}
