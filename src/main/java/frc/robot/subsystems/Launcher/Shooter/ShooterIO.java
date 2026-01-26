package frc.robot.subsystems.Launcher.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean left_connected = true;
    public double applied_volts_left = 0.0;
    public double tempCL = 0.0;

    public boolean right_connected = true;
    public double applied_volts_right = 0.0;
    public double tempCR = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runVoltsLeft(double volts) {}

  public default void runVoltsRight(double volts) {}

  public default void stop() {}

  public default void updatePID(double kp, double ki, double kd, double kg) {}
}
