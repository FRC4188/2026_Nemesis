package frc.robot.subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOInputs;

public class ClimberIOSim {
  private final DCMotorSim sim;
  private double applied_volts = 0;

  public ClimberIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 5, Constants.WristConstants.kGearRatio),
            DCMotor.getKrakenX60(1));
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.posRads = sim.getAngularPositionRad();
    inputs.tempC = 0;
    inputs.applied_volts = applied_volts;
  }

  public void runVolts(double volts) {
    MathUtil.clamp(volts, -12, 12);

    applied_volts = volts;
    sim.setInputVoltage(volts);
  }

  public double getAngle() {
    return sim.getAngularPositionRad();
  }
}
