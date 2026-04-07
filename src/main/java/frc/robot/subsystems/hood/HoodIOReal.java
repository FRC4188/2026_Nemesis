package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;

  private final TalonFXConfiguration motorConfigs;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> positionRots;
  private final StatusSignal<Temperature> tempC;

  private final Debouncer motorDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  public HoodIOReal() {
    motor = new TalonFX(Constants.Id.kHood, Constants.Robot.rio);

    motorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.HoodConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.HoodConstants.kSupplyCurrent))
            .withTorqueCurrent(new TorqueCurrentConfigs())
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.HoodConstants.kNuetralMode)
                    .withInverted(Constants.HoodConstants.kInvertedValue))
            .withSlot0(Constants.HoodConstants.hoodGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.HoodConstants.kGearRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0 / Constants.HoodConstants.kGearRatio)
                    .withMotionMagicAcceleration(1000.0 / Constants.HoodConstants.kGearRatio)
                    .withMotionMagicExpo_kV(0.12 * Constants.HoodConstants.kGearRatio)
                    .withMotionMagicExpo_kA(0.1));

    motor.getConfigurator().apply(motorConfigs);

    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    tempC = motor.getDeviceTemp();
    positionRots = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(5.0, appliedVolts, currentAmps, tempC);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, positionRots);

    motor.optimizeBusUtilization();
  }

  @Override
  public void setZero() {
    motor.setPosition(0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        motorDebouncer.calculate(
            BaseStatusSignal.refreshAll(appliedVolts, currentAmps, positionRots, tempC).isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.position = Rotation2d.fromRotations(positionRots.getValueAsDouble());
  }

  @Override
  public void setVolts(double output) {
    motor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(Rotation2d position) {
    motor.setControl(positionVoltageRequest.withPosition(position.getRotations()));
  }
}
