package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = true;
    public double leftTempC = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftVelocityRPM = 0.0;

    public boolean rightConnected = true;
    public double rightTempC = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightVelocityRPM = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVolts(double volts) {}

  public default void setTorqueCurrent(double amps) {}

  public default void setVelocity(double RPM) {}
}
