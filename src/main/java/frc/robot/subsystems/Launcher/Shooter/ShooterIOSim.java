package frc.robot.subsystems.Launcher.Shooter;

public class ShooterIOSim implements ShooterIO {

  private double leftTC = 0.0;
  private double rightTC = 0.0;
  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;

  public ShooterIOSim() {}

  @Override
  public void runVolts(double output) {
    leftTC = output;
    rightTC = output;
  }

  @Override
  public void setVelocity(double RPM) {
    leftVelocity = RPM;
    rightVelocity = RPM;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftCurrentAmps = leftTC;
    inputs.rightAppliedVolts = rightTC;
    inputs.leftVelocityRPM = leftVelocity;
    inputs.rightVelocityRPM = rightVelocity;
  }

  @Override
  public double getSetpoint() {
    return leftVelocity;
  }
}
