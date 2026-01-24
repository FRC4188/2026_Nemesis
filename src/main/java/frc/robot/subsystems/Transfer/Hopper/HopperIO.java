package frc.robot.subsystems.Transfer.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean connected = true;
    public double appliedVolts = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}
}
