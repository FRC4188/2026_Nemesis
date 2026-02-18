package frc.robot.subsystems.Launcher.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
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

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0);

  public HoodIOReal() {
    motor = new TalonFX(Constants.Id.kHood, Constants.Robot.rio);

    motorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.HoodConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.HoodConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.HoodConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.HoodConstants.kPeakReverseTC))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.HoodConstants.kNuetralMode)
                    .withInverted(Constants.HoodConstants.kInvertedValue))
            .withSlot0(Constants.HoodConstants.hoodGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(Constants.Id.kHoodCANCoder)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withRotorToSensorRatio(Constants.HoodConstants.kGearBox)
                    .withSensorToMechanismRatio(Constants.HoodConstants.kSproket))
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
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        motorDebouncer.calculate(
            BaseStatusSignal.refreshAll(appliedVolts, currentAmps, positionRots, tempC).isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.posRads = positionRots.getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(
        switch (Constants.IndexerConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void updatePID(double kP, double kI, double kD, double kV) {
    motorConfigs.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKG(kV)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    motor.getConfigurator().apply(motorConfigs);
  }

  @Override
  public void setPosition(Rotation2d radians) {
    motor.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(radians.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentFOC.withPosition(radians.getRotations());
        });
  }

  @Override
  public double getSetpoint() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }
}
