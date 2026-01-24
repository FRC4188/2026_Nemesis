package frc.robot.subsystems.Transfer.Hopper;

public class HopperIOSim implements HopperIO {
  // for some reason this did not make a DCMotorSim - Check into why later
  private double applied_volts = 0;

  public HopperIOSim() {
    applied_volts = 0;
  }

  @Override
  public void runVolts(double volts) {
    applied_volts = volts;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.appliedVolts = applied_volts;
  }
}
