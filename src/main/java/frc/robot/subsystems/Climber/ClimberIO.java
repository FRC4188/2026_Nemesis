package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = true;
    public double posMeters = 0.0;
    public double velMeters = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVolts(double output) {}

  public default void setPosition(double position) {}

  public default void setZero() {}
}
