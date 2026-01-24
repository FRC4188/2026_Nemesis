package frc.robot.subsystems.Launcher.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;

    public double angleRads = 0.0;

    public double appliedVolts = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default double getAngle() {
    return 0;
  }

  //   public default double getSetpoint() {
  //     return 0;
  //   }
}
