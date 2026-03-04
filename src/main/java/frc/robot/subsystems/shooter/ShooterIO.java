package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = true;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTempC = 0.0;
    public double leftVelocityRPM = 0.0;

    public boolean rightConnected = true;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTempC = 0.0;
    public double rightVelocityRPM = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void setVelocity(double rpm) {}

  public default double getSetpoint() {
    return 0;
  }
}
