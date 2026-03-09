package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOSim implements WristIO {
  // private final SingleJointedArmSim wSim;
  private double applidVolts;
  private double currentAmps;
  private Rotation2d position;

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

    inputs.appliedVolts = applidVolts;
    inputs.tempC = 0.0;
    inputs.currentAmps = currentAmps;
    inputs.position = position;
    inputs.velocity = Rotation2d.fromDegrees(10);
  }

  public void setVolts(double volts) {
    applidVolts = volts;
  }

  public void setTorqueCurrent(double amps) {
    currentAmps = amps;
  }

  public void setPosition(Rotation2d rotation) {
    position = rotation;
  }
}
