package frc.robot.subsystems.Loader.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = true;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default boolean isStalled() {
    return false;
  }
}
