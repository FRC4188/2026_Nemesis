package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HopperIOReal implements HopperIO {
  private final TalonFX aggitateMotor;
  private final TalonFX indexerMotor;
  private final TalonFXConfiguration a_config;
  private final TalonFXConfiguration i_config;

  private final StatusSignal<Voltage> aggitateAppliedVolts;
  private final StatusSignal<Current> aggitateCurrentAmps;
  private final StatusSignal<Temperature> aggitateTempC;

  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerCurrentAmps;
  private final StatusSignal<Temperature> indexerTempC;

  private final Debouncer aggitateDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer indexerDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  public HopperIOReal() {
    indexerMotor = new TalonFX(Constants.Id.kIndexer, Constants.Robot.rio);
    aggitateMotor = new TalonFX(Constants.Id.kHopper, Constants.Robot.rio);

    i_config =
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
    a_config =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.HopperConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.HopperConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.HopperConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.IndexerConstants.kPeakReverseTC))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.HopperConstants.kNuetralMode)
                    .withInverted(Constants.HopperConstants.kInvertedValue));

    aggitateMotor.getConfigurator().apply(a_config);
    indexerMotor.getConfigurator().apply(i_config);

    aggitateAppliedVolts = aggitateMotor.getMotorVoltage();
    aggitateCurrentAmps = aggitateMotor.getStatorCurrent();
    aggitateTempC = aggitateMotor.getDeviceTemp();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerCurrentAmps = indexerMotor.getStatorCurrent();
    indexerTempC = indexerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        5.0,
        aggitateAppliedVolts,
        aggitateCurrentAmps,
        aggitateTempC,
        indexerAppliedVolts,
        indexerCurrentAmps,
        indexerTempC);

    aggitateMotor.optimizeBusUtilization();
    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void setAggitateVolts(double volts) {
    aggitateMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setIndexerVolts(double volts) {
    indexerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.aggitateConnected =
        aggitateDebouncer.calculate(
            BaseStatusSignal.refreshAll(aggitateAppliedVolts, aggitateCurrentAmps, aggitateTempC)
                .isOK());
    inputs.indexerConnected =
        indexerDebouncer.calculate(
            BaseStatusSignal.refreshAll(indexerAppliedVolts, indexerCurrentAmps, indexerTempC)
                .isOK());
    inputs.aggitateAppliedVolts = aggitateAppliedVolts.getValueAsDouble();
    inputs.aggitateCurrentAmps = aggitateCurrentAmps.getValueAsDouble();
    inputs.aggitateTempC = aggitateTempC.getValueAsDouble();
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerCurrentAmps = indexerCurrentAmps.getValueAsDouble();
    inputs.indexerTempC = indexerTempC.getValueAsDouble();
  }
}
