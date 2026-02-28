package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean aggitateConnected = true;
    public double aggitateAppliedVolts = 0.0;
    public double aggitateCurrentAmps = 0.0;
    public double aggitateTempC = 0.0;

    public boolean indexerConnected = true;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;
    public double indexerTempC = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void runAggitateVolts(double volts) {}

  public default void runIndexerVolts(double volts) {}
}
