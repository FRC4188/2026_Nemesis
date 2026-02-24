package frc.robot.subsystems.Transfer.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = true;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
