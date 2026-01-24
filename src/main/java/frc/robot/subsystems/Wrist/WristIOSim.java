package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {
  private double applied_volts;
  private final SingleJointedArmSim wSim;

  public WristIOSim() {
    wSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            Constants.WristConstants.kGearRatio,
            0.1,
            1,
            0 + Math.PI / 2, // these numbers must change
            1.35 + Math.PI / 2, // these numbers must change
            true,
            0,
            null); // not sure what the last argument is, look into it
  }

  @Override
  public void runVolts(double volts) {
    wSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0);
    }
    wSim.update(0.02);
    inputs.appliedVolts = applied_volts;
    inputs.posRads = wSim.getAngleRads();
  }

  @Override
  public double getAngle() {
    return wSim.getAngleRads() - Math.PI / 2;
  }
}
