package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX leftLeader;
  private final TalonFX left2Follow;
  private final TalonFX left3Follow;
  private final TalonFX rightFollow;

  private final TalonFXConfiguration shooterConfigs;

  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> leftCurrentAmps;
  private final StatusSignal<Current> rightCurrentAmps;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Temperature> leftTempC;
  private final StatusSignal<Temperature> rightTempC;

  private final StatusSignal<Voltage> left2AppliedVolts;
  private final StatusSignal<Voltage> left3AppliedVolts;
  private final StatusSignal<Current> left2CurrentAmps;
  private final StatusSignal<Current> left3CurrentAmps;
  private final StatusSignal<AngularVelocity> left2Velocity;
  private final StatusSignal<AngularVelocity> left3Velocity;
  private final StatusSignal<Temperature> left2TempC;
  private final StatusSignal<Temperature> left3TempC;

  private final Debouncer leftDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer left2Debouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer left3Debouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rightDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  public ShooterIOReal() {
    leftLeader = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
    left2Follow = new TalonFX(Constants.Id.kLeftShoot2, Constants.Robot.rio);
    left3Follow = new TalonFX(Constants.Id.kLeftShoot3, Constants.Robot.rio);
    rightFollow = new TalonFX(Constants.Id.kRightShooter, Constants.Robot.rio);

    shooterConfigs =
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

    leftLeader.getConfigurator().apply(shooterConfigs);
    left2Follow.getConfigurator().apply(shooterConfigs);
    left3Follow.getConfigurator().apply(shooterConfigs);
    rightFollow.getConfigurator().apply(shooterConfigs);

    rightFollow.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    left2Follow.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    left3Follow.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    leftAppliedVolts = leftLeader.getMotorVoltage();
    rightAppliedVolts = rightFollow.getMotorVoltage();
    leftCurrentAmps = leftLeader.getStatorCurrent();
    rightCurrentAmps = rightFollow.getStatorCurrent();
    leftVelocity = leftLeader.getVelocity();
    rightVelocity = rightFollow.getVelocity();
    leftTempC = leftLeader.getDeviceTemp();
    rightTempC = rightFollow.getDeviceTemp();
    left2AppliedVolts = left2Follow.getMotorVoltage();
    left3AppliedVolts = left3Follow.getMotorVoltage();
    left2CurrentAmps = left2Follow.getStatorCurrent();
    left3CurrentAmps = left3Follow.getStatorCurrent();
    left2Velocity = left2Follow.getVelocity();
    left3Velocity = left3Follow.getVelocity();
    left2TempC = left2Follow.getDeviceTemp();
    left3TempC = left3Follow.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftAppliedVolts,
        leftCurrentAmps,
        rightAppliedVolts,
        rightCurrentAmps,
        leftTempC,
        rightTempC,
        leftVelocity,
        rightVelocity,
        left2AppliedVolts,
        left2CurrentAmps,
        left3AppliedVolts,
        left3CurrentAmps,
        left2TempC,
        left3TempC,
        left2Velocity,
        left3Velocity);

    BaseStatusSignal.setUpdateFrequencyForAll(5.0, leftTempC, rightTempC, left2TempC, left3TempC);

    leftLeader.optimizeBusUtilization();
    left2Follow.optimizeBusUtilization();
    left3Follow.optimizeBusUtilization();
    rightFollow.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftConnected =
        leftDebouncer.calculate(
            BaseStatusSignal.refreshAll(leftAppliedVolts, leftCurrentAmps, leftVelocity, leftTempC)
                .isOK());

    inputs.rightConnected =
        rightDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    rightAppliedVolts, rightCurrentAmps, rightVelocity, rightTempC)
                .isOK());

    inputs.left2Connected =
        left2Debouncer.calculate(
            BaseStatusSignal.refreshAll(
                    left2AppliedVolts, left2CurrentAmps, left2Velocity, left2TempC)
                .isOK());

    inputs.left3Connected =
        left3Debouncer.calculate(
            BaseStatusSignal.refreshAll(
                    left3AppliedVolts, left3CurrentAmps, left3Velocity, left3TempC)
                .isOK());

    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.leftTempC = leftTempC.getValueAsDouble();
    inputs.rightTempC = rightTempC.getValueAsDouble();
    inputs.leftVelocityRPM = leftVelocity.getValueAsDouble() * 60.0;
    inputs.rightVelocityRPM = rightVelocity.getValueAsDouble() * 60.0;

    inputs.left2AppliedVolts = left2AppliedVolts.getValueAsDouble();
    inputs.left3AppliedVolts = left3AppliedVolts.getValueAsDouble();
    inputs.left2CurrentAmps = left2CurrentAmps.getValueAsDouble();
    inputs.left3CurrentAmps = left3CurrentAmps.getValueAsDouble();
    inputs.left2TempC = left2TempC.getValueAsDouble();
    inputs.left3TempC = left3TempC.getValueAsDouble();
    inputs.left2VelocityRPM = left2Velocity.getValueAsDouble() * 60.0;
    inputs.left3VelocityRPM = left3Velocity.getValueAsDouble() * 60.0;
  }

  @Override
  public void setVolts(double volts) {
    leftLeader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setTorqueCurrent(double amps) {
    leftLeader.setControl(torqueCurrentFOC.withOutput(amps));
  }

  @Override
  public void setVelocity(double RPM) {
    if (RPM == 0.0) {
      setTorqueCurrent(0);
      return;
    }
    leftLeader.setControl(velocityTorqueCurrentFOC.withVelocity(RPM / 60.0));
  }
}
