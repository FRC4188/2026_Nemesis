package frc.robot.subsystems.Loader.Wrist;

import edu.wpi.first.math.geometry.Rotation2d;
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

  default void setPosition(Rotation2d rotation) {}

  default void updatePID(double kp, double ki, double kd, double kg) {}
}
