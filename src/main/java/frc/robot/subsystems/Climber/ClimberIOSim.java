package frc.robot.subsystems.climber;

public class ClimberIOSim implements ClimberIO {
  // private final DCMotorSim sim;

  private double appliedVolts = 0.0;
  private double position = 0.0;

  public ClimberIOSim() {
    // sim =
    //     new DCMotorSim(
    //         LinearSystemId.createDCMotorSystem(
    //             DCMotor.getKrakenX60(1), 5, Constants.ClimberConstants.kGearRatio),
    //         DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    inputs.posMeters = position;
  }

  @Override
  public void setVolts(double output) {
    appliedVolts = output;
    position = -1;
  }

  @Override
  public void setPosition(double position) {
    this.position = position;
    appliedVolts = 0.0;
  }

}
