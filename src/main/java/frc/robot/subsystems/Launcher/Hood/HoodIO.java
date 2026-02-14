package frc.robot.subsystems.Launcher.Hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = true;
    public boolean coderConnected = true;

    public double angleRads = 0.0;
    public double appliedVolts = 0.0;
    public double canPos = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setPosition(Rotation2d radians) {}

  public default double getSetpoint() {
    return 0.0;
  }

  public default void updatePID(double kp, double ki, double kd, double kg) {}
}
