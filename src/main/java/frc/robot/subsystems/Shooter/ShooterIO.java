package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = true;
    public double applied_volts = 0.0;
    public double tempC1 = 0.0;
    public double tempC2 = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runVoltsLeft(double volts) {}

  public default void runVoltsRight(double volts) {}

  public default void stop() {}
}
