package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {

  // private double appliedVolts = 0.0;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  // private double RPM = 0.0;
  private double leftRPM = 0.0;
  private double rightRPM = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftVelocityRPM = leftRPM;
    inputs.rightVelocityRPM = rightRPM;

    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.rightAppliedVolts = rightAppliedVolts;
  }

  @Override
  public void setVolts(double volts) {
    leftRPM = -100;
    rightRPM = -100;
    leftAppliedVolts = volts;
    rightAppliedVolts = volts;
  }

  @Override
  public void setVelocity(double rpm) {
    leftAppliedVolts = 0.0;
    rightAppliedVolts = 0.0;
    leftRPM = rpm;
  }
}
