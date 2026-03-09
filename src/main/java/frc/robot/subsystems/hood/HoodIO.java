package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = true;

    public Rotation2d position = Rotation2d.kZero;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
    public Rotation2d velocity = Rotation2d.kZero;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setVolts(double output) {}

  public default void setPosition(Rotation2d position) {}

  public default void setZero() {}
}
