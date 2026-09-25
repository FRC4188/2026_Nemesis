package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = true;

    public double appliedVolts = 0.0;
    public double tempC = 0.0;
    public double currentAmps = 0.0;

    public Rotation2d position = Rotation2d.kZero;
    public AngularVelocity motorVelocity =
        AngularVelocity.ofRelativeUnits(0, Units.DegreesPerSecond);
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setVolts(double volts) {}

  default void setCoast() {}

  default void setPosition(Rotation2d rotation) {}

  default void setZero() {}
}
