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
    public double leftAccelerationRPMPM = 0.0;

    public boolean rightConnected = true;
    public double rightTempC = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightVelocityRPM = 0.0;
    public double rightAccelerationRPMPM = 0.0;

    public boolean left2Connected = true;
    public double left2TempC = 0.0;
    public double left2AppliedVolts = 0.0;
    public double left2CurrentAmps = 0.0;
    public double left2VelocityRPM = 0.0;
    public double left2AccelerationRPMPM = 0.0;

    public boolean left3Connected = true;
    public double left3TempC = 0.0;
    public double left3AppliedVolts = 0.0;
    public double left3CurrentAmps = 0.0;
    public double left3VelocityRPM = 0.0;
    public double left3AccelerationRPMPM = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVolts(double volts) {}

  public default void setTorqueCurrent(double amps) {}

  public default void setVelocity(double RPM) {}
}
