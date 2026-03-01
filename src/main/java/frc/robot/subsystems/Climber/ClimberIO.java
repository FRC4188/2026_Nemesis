package frc.robot.subsystems.Climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = true;
    public double posRots = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runVolts(double output) {}

  public default void setPosition(Rotation2d rotations) {}

  public default double getSetpoint() {
    return 0;
  }
}
