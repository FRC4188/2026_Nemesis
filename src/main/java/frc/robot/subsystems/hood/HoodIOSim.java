package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodIOSim implements HoodIO {
  private double appliedVolts = 0;
  private Rotation2d rotation = Rotation2d.kZero;
  // private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);

  //  private final DCMotorSim hoodSim;

  // arbitrary values
  // private static final double HOOD_KP = 0.8;
  // private static final double HOOD_KD = 0.0;
  // private PIDController hoodController = new PIDController(HOOD_KP, 0, HOOD_KD);

  public HoodIOSim() {
    // hoodSim =
    //     new DCMotorSim(
    //         LinearSystemId.createDCMotorSystem(
    //             HOOD_GEARBOX, 0.004, Constants.HoodConstants.kGearRatio),
    //         HOOD_GEARBOX);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    inputs.position = rotation;
  }

  @Override
  public void setVolts(double output) {
    appliedVolts = output;
    rotation = Rotation2d.kCCW_90deg;
  }

  @Override
  public void setPosition(Rotation2d position) {
    this.rotation = position;
    appliedVolts = 0.0;
  }
}
