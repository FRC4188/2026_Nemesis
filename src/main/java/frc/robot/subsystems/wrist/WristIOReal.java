package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

public class WristIOReal implements WristIO {
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;
  private final StatusSignal<AngularVelocity> velocityRots;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  public WristIOReal() {
    motor = new TalonFX(Constants.Id.kWrist, Constants.Robot.rio);

    motorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.WristConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.WristConstants.kStatorCurrent))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.WristConstants.kNuetralMode)
                    .withInverted(Constants.WristConstants.kInvertedValue))
            .withSlot0(Constants.WristConstants.wristGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.WristConstants.kGearRatio))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0 / Constants.WristConstants.kGearRatio)
                    .withMotionMagicAcceleration(1000.0 / Constants.WristConstants.kGearRatio)
                    .withMotionMagicExpo_kV(0.12 * Constants.WristConstants.kGearRatio)
                    .withMotionMagicExpo_kA(0.1));

    motor.getConfigurator().apply(motorConfig);

    posRots = motor.getPosition();
    tempC = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRots = motor.getVelocity();

    posRots.setUpdateFrequency(50.0);
    appliedVolts.setUpdateFrequency(5.0);
    tempC.setUpdateFrequency(5.0);
    currentAmps.setUpdateFrequency(5.0);
    velocityRots.setUpdateFrequency(5.0);

    motor.optimizeBusUtilization();
    motor.setPosition(Constants.WristConstants.Max_A.getRotations());
  }

  @Override
  public void setVolts(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setZero() {
    motor.setPosition(0.0);
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    motor.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(appliedVolts, tempC, posRots, currentAmps);

    inputs.connected = motorConnectedDebouncer.calculate(motorStatus.isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.position = Rotation2d.fromRotations(posRots.getValueAsDouble());
    inputs.velocity = Rotation2d.fromRotations(velocityRots.getValueAsDouble());
  }
}
