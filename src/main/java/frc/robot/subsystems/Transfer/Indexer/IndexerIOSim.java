package frc.robot.subsystems.Transfer.Indexer;

public class IndexerIOSim implements IndexerIO {

  private double appliedVolts;

  public IndexerIOSim() {}

  @Override
  public void setOpenLoop(double output) {
    appliedVolts = output;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }
}
