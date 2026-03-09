package frc.robot.subsystems.hopper;

public class HopperIOSim implements HopperIO {
  private double aAppliedVolts;
  private double iAppliedVolts;

  public HopperIOSim() {}

  public void updateInputs(HopperIOInputs inputs) {
    inputs.aggitateAppliedVolts = aAppliedVolts;
    inputs.indexerAppliedVolts = iAppliedVolts;
  }

  public void setAggitateVolts(double volts) {
    aAppliedVolts = volts;
  }

  public void setIndexerVolts(double volts) {
    iAppliedVolts = volts;
  }
}
