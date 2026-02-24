package frc.robot.subsystems.Transfer.Indexer;

public class IndexerIOSim implements IndexerIO {

  private double appliedVolts;

  public IndexerIOSim() {}

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }
}
