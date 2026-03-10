package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOSim implements WristIO {
  // private final SingleJointedArmSim wSim;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  private Rotation2d position = Rotation2d.fromDegrees(144);

  public WristIOSim() {
    // wSim =
    //     new SingleJointedArmSim(
    //         DCMotor.getKrakenX60(1),
    //         Constants.WristConstants.kGearRatio,
    //         0.1,
    //         1,
    //         Constants.WristConstants.Min_A.getRadians(),
    //         Constants.WristConstants.Max_A.getRadians(),
    //         true,
    //         Constants.WristConstants.Max_A.getRadians());
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.connected = true;

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    inputs.position = position;
  }

  public void setVolts(double volts) {
    currentAmps = 0.0;
    appliedVolts = volts;
    position = Rotation2d.k180deg;
  }

  public void setTorqueCurrent(double amps) {
    currentAmps = amps;
    appliedVolts = 0.0;
    position = Rotation2d.k180deg;
  }

  public void setPosition(Rotation2d rotation) {
    position = rotation;
    appliedVolts = 0.0;
    currentAmps = 0.0;
  }
}
