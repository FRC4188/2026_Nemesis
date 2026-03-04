package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class WristIOReal implements WristIO {
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  public WristIOReal() {
    motor = new TalonFX(Constants.Id.kWrist, Constants.Robot.rio);

    motorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.WristConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.WristConstants.kStatorCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.WristConstants.kNuetralMode)
                    .withInverted(Constants.WristConstants.kInvertedValue))
            .withSlot0(Constants.WristConstants.wristGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.WristConstants.kGearRatio))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.WristConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.WristConstants.kPeakReverseTC))
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

    posRots.setUpdateFrequency(Hertz.of(50.0));
    appliedVolts.setUpdateFrequency(Hertz.of(5.0));
    tempC.setUpdateFrequency(Hertz.of(5.0));
    currentAmps.setUpdateFrequency(5.0);

    BaseStatusSignal.setUpdateFrequencyForAll(5.0, appliedVolts, currentAmps, tempC);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, posRots);

    motor.optimizeBusUtilization();
    motor.setPosition(Units.radiansToRotations(Constants.WristConstants.Max_A));
  }

  @Override
  public void runVolts(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public boolean isStalled() {
    return Math.abs(motor.getStatorCurrent().getValueAsDouble())
            > Constants.WristConstants.kStallCurrent
        && Math.abs(motor.getVelocity().getValueAsDouble()) < 0.5;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(appliedVolts, tempC, posRots, currentAmps);

    inputs.connected = motorConnectedDebouncer.calculate(motorStatus.isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.posRads = posRots.getValueAsDouble() * Math.PI * 2;
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    motor.setControl(
        switch (Constants.WristConstants.motorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(
              rotation.getRotations());
        });
  }

  public double getSetpoint() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }
}
