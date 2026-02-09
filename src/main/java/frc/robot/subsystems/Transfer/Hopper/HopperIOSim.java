package frc.robot.subsystems.Transfer.Hopper;

public class HopperIOSim implements HopperIO {
  private double appliedVolts;

  public HopperIOSim() {}

  @Override
  public void setOpenLoop(double output) {
    appliedVolts = output;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }
}
