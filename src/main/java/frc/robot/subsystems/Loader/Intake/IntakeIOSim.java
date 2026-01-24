package frc.robot.subsystems.Loader.Intake;

public class IntakeIOSim implements IntakeIO {
  private double applied_volts = 0;

  public IntakeIOSim() {
    applied_volts = 0;
  }

  @Override
  public void runVolts(double volts) {
    applied_volts = volts;
  }

  @Override
  public void stop() {
    applied_volts = 0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.appliedVolts = applied_volts;
  }
}
