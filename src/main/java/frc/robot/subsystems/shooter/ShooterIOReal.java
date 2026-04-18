package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX leftLeader;
  private final TalonFX rightFollow;

  private final TalonFXConfiguration leftShooterConfigs;
  private final TalonFXConfiguration rightShooterConfigs;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Temperature> leftTempC;
  private final StatusSignal<Temperature> rightTempC;

  private final Debouncer leftDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rightDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  public ShooterIOReal() {
    leftLeader = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
    rightFollow = new TalonFX(Constants.Id.kRightShooter, Constants.Robot.rio);

    leftShooterConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kSupplyCurrent))
            .withVoltage(
                new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.ShooterConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.ShooterConstants.kPeakReverseTC))
            .withSlot0(Constants.ShooterConstants.shooterGains)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.ShooterConstants.kNuetralMode)
                    .withInverted(Constants.ShooterConstants.kLeftInvertedValue))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(Constants.ShooterConstants.kGearRatio));

    rightShooterConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kSupplyCurrent))
            .withVoltage(
                new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.ShooterConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.ShooterConstants.kPeakReverseTC))
            .withSlot0(Constants.ShooterConstants.shooterGains)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.ShooterConstants.kNuetralMode)
                    .withInverted(Constants.ShooterConstants.kRightInvertedValue))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(Constants.ShooterConstants.kGearRatio));

    leftLeader.getConfigurator().apply(leftShooterConfigs);
    rightFollow.getConfigurator().apply(rightShooterConfigs);

    appliedVolts = leftLeader.getMotorVoltage();
    currentAmps = leftLeader.getStatorCurrent();
    leftTempC = leftLeader.getDeviceTemp();
    velocity = leftLeader.getVelocity();
    rightTempC = rightFollow.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        5.0, appliedVolts, currentAmps, leftTempC, rightTempC);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity);

    leftLeader.optimizeBusUtilization();
    rightFollow.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftConnected =
        leftDebouncer.calculate(
            BaseStatusSignal.refreshAll(appliedVolts, currentAmps, velocity, leftTempC).isOK());

    inputs.rightConnected =
        rightDebouncer.calculate(BaseStatusSignal.refreshAll(rightTempC).isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.leftTempC = leftTempC.getValueAsDouble();
    inputs.rightTempC = rightTempC.getValueAsDouble();
    inputs.velocityRPM = velocity.getValueAsDouble() * 60.0;
  }

  @Override
  public void setVolts(double volts) {
    leftLeader.setControl(voltageRequest.withOutput(volts));
    rightFollow.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setTorqueCurrent(double amps) {
    leftLeader.setControl(torqueCurrentFOC.withOutput(amps));
    rightFollow.setControl(torqueCurrentFOC.withOutput(amps));
  }

  @Override
  public void setVelocity(double RPM) {
    if (RPM == 0.0) {
      setTorqueCurrent(0);
    }

    leftLeader.setControl(velocityTorqueCurrentFOC.withVelocity(RPM / 60.0));
    rightFollow.setControl(velocityTorqueCurrentFOC.withVelocity(RPM / 60.0));
  }
}
