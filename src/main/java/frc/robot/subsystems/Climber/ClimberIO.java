package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = true;
    public double posRads = 0.0;
    public double applied_volts = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default double getAngle() {
    return 0;
  }
}
