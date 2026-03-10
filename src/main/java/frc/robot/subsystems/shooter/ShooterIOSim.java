package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {

  private double appliedVolts = 0.0;
  private double RPM = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftVelocityRPM = RPM;
    inputs.rightVelocityRPM = RPM;

    inputs.leftAppliedVolts = appliedVolts;
    inputs.rightAppliedVolts = appliedVolts;
  }

  @Override
  public void setVolts(double volts) {
    RPM = -100;
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double rpm) {
    appliedVolts = 0.0;
    RPM = rpm;
  }
}
