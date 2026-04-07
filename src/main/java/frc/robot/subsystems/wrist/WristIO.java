package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = true;

    public double appliedVolts = 0.0;
    public double tempC = 0.0;
    public double currentAmps = 0.0;

    public Rotation2d position = Rotation2d.kZero;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setVolts(double volts) {}

  default void setPosition(Rotation2d rotation) {}

  default void setZero() {}
}
