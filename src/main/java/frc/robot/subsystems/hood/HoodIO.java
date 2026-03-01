package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = true;

    public double posRads = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void runVolts(double output) {}

  public default void setPosition(Rotation2d radians) {}

  public default double getSetpoint() {
    return 0.0;
  }
}
