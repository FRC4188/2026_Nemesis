package frc.robot.subsystems.Superstructure.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = true;
    public double appliedVolts = 0.0;
    public double tempC = 0.0;
    public double posRads = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void runVolts(double volts) {}

  default double getAngle() {
    return 0;
  }
}
