package frc.robot.subsystems.Loader.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {

  private final SingleJointedArmSim wSim;

  public WristIOSim() {
    wSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.WristConstants.kGearRatio,
            0.1,
            1,
            0.0,
            Units.degreesToRadians(120),
            true,
            Units.degreesToRadians(0));
  }

  @Override
  public void runVolts(double volts) {

    wSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    wSim.setState(rotation.getRadians(), 1);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    wSim.update(0.02);
    // inputs.appliedVolts = wSim.get;
    inputs.posRads = wSim.getAngleRads();
  }

  public double getAngle() {
    return wSim.getAngleRads() - Math.PI / 2;
  }
}
