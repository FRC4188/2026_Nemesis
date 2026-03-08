package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = true;

    public double appliedVolts = 0.0;
    public double tempC = 0.0;
    public double posRads = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void runTorqueCurrent(double amps) {}

  default void setPosition(Rotation2d rotation) {}

  default double getSetpoint() {
    return 0;
  }

  default boolean isStalled() {
    return false;
  }

  default void zero() {}
}
