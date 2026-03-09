package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
  private double appliedVolts = 0.0;

  public IntakeIOSim() {
    appliedVolts = 0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = volts;
  }
}
