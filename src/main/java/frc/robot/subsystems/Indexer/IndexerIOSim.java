package frc.robot.subsystems.Indexer;

public class IndexerIOSim implements IndexerIO {

  private double applied_volts = 0;

  public IndexerIOSim() {
    applied_volts = 0;
  }

  @Override
  public void runVolts(double volts) {
    applied_volts = volts;
  }

  @Override
  public boolean isStalled() {
    return false;
  }

  @Override
  public void UpdateInputs(IndexerIOInputs inputs) {
    inputs.applied_volts = applied_volts;
  }
}
