package frc.robot.subsystems.Launcher.Shooter;

public class ShooterIOSim implements ShooterIO {

  private double applied_volts_left = 0;
  private double applied_volts_right = 0;

  public ShooterIOSim() {
    applied_volts_left = 0;
    applied_volts_right = 0;
  }

  @Override
  public void runVoltsLeft(double volts) {
    applied_volts_left = volts;
  }

  @Override
  public void runVoltsRight(double volts) {
    applied_volts_right = volts;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.applied_volts_left = applied_volts_left;
    inputs.applied_volts_right = applied_volts_right;
  }
}
