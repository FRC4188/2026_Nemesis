package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = true;
    public double applied_volts = 0.0;
    public double tempC = 0.0;
  }

  public default void UpdateInputs(IndexerIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default boolean isStalled() {
    return false;
  }

  public default void stop() {}
}
