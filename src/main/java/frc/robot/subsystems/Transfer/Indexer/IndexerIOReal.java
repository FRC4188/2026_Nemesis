package frc.robot.subsystems.Transfer.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Temperature> tempC;

  private final Debouncer debouncer = new Debouncer(0.5, DebounceType.kFalling);

  public IndexerIOReal() {
    motor = new TalonFX(Constants.Id.kIndexer, Constants.Robot.rio);
    config =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.IndexerConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.IndexerConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.IndexerConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.IndexerConstants.kPeakReverseTC))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.IndexerConstants.kNuetralMode)
                    .withInverted(Constants.IndexerConstants.kInvertedValue));

    motor.getConfigurator().apply(config);

    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    tempC = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(5.0, appliedVolts, currentAmps, tempC);

    motor.optimizeBusUtilization();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected =
        debouncer.calculate(BaseStatusSignal.refreshAll(appliedVolts, currentAmps, tempC).isOK());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
  }
}
