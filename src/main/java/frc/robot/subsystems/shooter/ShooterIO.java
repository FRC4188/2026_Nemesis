package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = true;
    public double leftTempC = 0.0;

    public boolean rightConnected = true;
    public double rightTempC = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocityRPM = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVolts(double volts) {}

  public default void setTorqueCurrent(double amps) {}

  public default void setVelocity(double RPM) {}
}
