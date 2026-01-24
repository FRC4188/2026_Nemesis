package frc.robot.subsystems.Transfer.Indexer;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IndexerIOReal implements IndexerIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempC;
  private final DigitalInput
      breaker; // this might be useless, because it would take a very specific situation to stall
  // indexer. Might get rid of this eventually

  public IndexerIOReal() {
    motor = new TalonFX(Constants.Id.kIndexer, Constants.Robot.rio);

    breaker = new DigitalInput(0);

    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(IntakeConstants.kMotorConfig); // change the motor config later
    appliedVolts = motor.getMotorVoltage();
    tempC = motor.getDeviceTemp();
    motor.optimizeBusUtilization();

    // arbitrary hertz values
    appliedVolts.setUpdateFrequency(Hertz.of(50));
    tempC.setUpdateFrequency(Hertz.of(0.5));
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public boolean isStalled() {
    return !breaker
        .get(); // true when the breaker is not broken (again might not be needed, but taken from
    // Argo intake)
  }

  @Override
  public void UpdateInputs(IndexerIOInputs inputs) {
    inputs.applied_volts = appliedVolts.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
  }
}
