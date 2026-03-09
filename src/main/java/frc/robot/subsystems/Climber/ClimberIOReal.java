package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;
  private final StatusSignal<AngularVelocity> velRots;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  private final Debouncer debouncer = new Debouncer(0.5, DebounceType.kFalling);

  public ClimberIOReal() {
    motor = new TalonFX(Constants.Id.kClimber, Constants.Robot.rio);

    motorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ClimberConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.ClimberConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.ClimberConstants.kNuetralMode)
                    .withInverted(Constants.ClimberConstants.kInvertedValue))
            .withSlot0(Constants.ClimberConstants.climberGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.ClimberConstants.kConversion))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0 / Constants.ClimberConstants.kConversion)
                    .withMotionMagicAcceleration(1000.0 / Constants.ClimberConstants.kConversion)
                    .withMotionMagicExpo_kV(0.12 / Constants.ClimberConstants.kConversion)
                    .withMotionMagicExpo_kA(0.1));

    motor.getConfigurator().apply(motorConfig);

    posRots = motor.getPosition();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    tempC = motor.getDeviceTemp();
    velRots = motor.getVelocity();

    posRots.setUpdateFrequency(Hertz.of(50.0));
    appliedVolts.setUpdateFrequency(Hertz.of(5.0));
    currentAmps.setUpdateFrequency(Hertz.of(5.0));
    tempC.setUpdateFrequency(Hertz.of(5.0));
    velRots.setUpdateFrequency(Hertz.of(5.0));

    motor.optimizeBusUtilization();
  }

  public void updateInputs(ClimberIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(posRots, appliedVolts, tempC, velRots);

    inputs.connected = debouncer.calculate(status.isOK());

    inputs.posMeters = posRots.getValueAsDouble();
    inputs.velMeters = velRots.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
  }

  @Override
  public void setVolts(double output) {
    motor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(double position) {
    motor.setControl(positionTorqueCurrentRequest.withPosition(position));
  }

  @Override
  public void setZero() {
    motor.setPosition(0.0);
  }
}
