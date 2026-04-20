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

  private final Debouncer leftDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rightDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  public ShooterIOReal() {
    leftLeader = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
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
    rightFollow.getConfigurator().apply(shooterConfigs);

    rightFollow.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    leftAppliedVolts = leftLeader.getMotorVoltage();
    rightAppliedVolts = rightFollow.getMotorVoltage();
    leftCurrentAmps = leftLeader.getStatorCurrent();
    rightCurrentAmps = rightFollow.getStatorCurrent();
    leftVelocity = leftLeader.getVelocity();
    rightVelocity = rightFollow.getVelocity();
    leftTempC = leftLeader.getDeviceTemp();
    rightTempC = rightFollow.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        5.0,
        leftAppliedVolts,
        leftCurrentAmps,
        rightAppliedVolts,
        rightCurrentAmps,
        leftTempC,
        rightTempC);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftVelocity, rightVelocity);

    leftLeader.optimizeBusUtilization();
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

    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrentAmps.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrentAmps.getValueAsDouble();
    inputs.leftTempC = leftTempC.getValueAsDouble();
    inputs.rightTempC = rightTempC.getValueAsDouble();
    inputs.leftVelocityRPM = leftVelocity.getValueAsDouble() * 60.0;
    inputs.rightVelocityRPM = rightVelocity.getValueAsDouble() * 60.0;
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
    }

    leftLeader.setControl(velocityTorqueCurrentFOC.withVelocity(RPM / 60.0));
  }
}
