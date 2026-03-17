package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
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
  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  private final TalonFXConfiguration motorConfigs;

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
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  public ShooterIOReal() {
    motorLeft = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
    motorRight = new TalonFX(Constants.Id.kRightShooter, Constants.Robot.rio);

    motorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.ShooterConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.ShooterConstants.kPeakReverseTC))
            .withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(Constants.ShooterConstants.kNuetralMode))
            .withSlot0(Constants.ShooterConstants.shooterGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(Constants.ShooterConstants.kGearRatio));

    motorLeft
        .getConfigurator()
        .apply(
            motorConfigs.MotorOutput.withInverted(Constants.ShooterConstants.kLeftInvertedValue));
    motorRight
        .getConfigurator()
        .apply(
            motorConfigs.MotorOutput.withInverted(Constants.ShooterConstants.kRightInvertedValue));

    leftAppliedVolts = motorLeft.getMotorVoltage();
    rightAppliedVolts = motorRight.getMotorVoltage();
    leftCurrentAmps = motorLeft.getStatorCurrent();
    rightCurrentAmps = motorRight.getStatorCurrent();
    leftTempC = motorLeft.getDeviceTemp();
    rightTempC = motorRight.getDeviceTemp();
    leftVelocity = motorLeft.getVelocity();
    rightVelocity = motorRight.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        5.0,
        leftAppliedVolts,
        rightAppliedVolts,
        leftCurrentAmps,
        rightCurrentAmps,
        leftTempC,
        rightTempC);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftVelocity, rightVelocity);

    motorLeft.optimizeBusUtilization();
    motorRight.optimizeBusUtilization();
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
    motorRight.setControl(voltageRequest.withOutput(volts));
    motorLeft.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(double leftRPM, double rightRPM) {
    if (leftRPM == 0.0) {
      motorLeft.setControl(voltageRequest.withOutput(0.0));
    }

    if (rightRPM == 0.0) {
      motorRight.setControl(voltageRequest.withOutput(0.0));
    }

    motorRight.setControl(velocityTorqueCurrentFOC.withVelocity(rightRPM / 60.0));
    motorLeft.setControl(velocityTorqueCurrentFOC.withVelocity(leftRPM / 60.0));
  }
}
