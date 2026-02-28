package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.Robot;

public class HoodIOSim implements HoodIO {
  private double applied_volts = 0;
  private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim hoodSim;

  // arbitrary values
  private static final double HOOD_KP = 0.8;
  private static final double HOOD_KD = 0.0;
  private PIDController hoodController = new PIDController(HOOD_KP, 0, HOOD_KD);

  public HoodIOSim() {
    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HOOD_GEARBOX, 0.004, Constants.HoodConstants.kGearRatio),
            HOOD_GEARBOX);
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.appliedVolts = applied_volts;
    inputs.tempC = 0;
    inputs.posRads = getAngle();
    hoodSim.update(Robot.loopPeriodSecs);
  }

  public void runVolts(double volts) {
    applied_volts = MathUtil.clamp(volts, -12, 12);
    hoodSim.setInputVoltage(applied_volts);
  }

  public double getAngle() {
    return hoodSim.getAngularPositionRad();
  }

  public void runVolt() {
    applied_volts = hoodController.calculate(HOOD_KD);
    hoodSim.setInputVoltage(applied_volts);
  }
}
