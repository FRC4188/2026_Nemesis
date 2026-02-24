package frc.robot.subsystems.Launcher.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  private final TalonFXConfiguration leftMotorConfigs;
  private final TalonFXConfiguration rightMotorConfigs;

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

  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  public ShooterIOReal() {
    motorLeft = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
    motorRight = new TalonFX(Constants.Id.kRightShooter, Constants.Robot.rio);

    rightMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.ShooterConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.IndexerConstants.kPeakReverseTC))
            .withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(Constants.ShooterConstants.kNuetralMode))
            .withSlot0(Constants.ShooterConstants.rightShooterGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(Constants.ShooterConstants.kGearRatio));

    leftMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.ShooterConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.IndexerConstants.kPeakReverseTC))
            .withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(Constants.ShooterConstants.kNuetralMode))
            .withSlot0(Constants.ShooterConstants.leftShooterGains)
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(Constants.ShooterConstants.kGearRatio));

    motorLeft
        .getConfigurator()
        .apply(
            leftMotorConfigs.MotorOutput.withInverted(
                Constants.ShooterConstants.kLeftInvertedValue));
    motorRight
        .getConfigurator()
        .apply(
            rightMotorConfigs.MotorOutput.withInverted(
                Constants.ShooterConstants.kRightInvertedValue));

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
  public void runVolts(double volts) {
    motorLeft.setVoltage(volts);
    motorRight.setVoltage(volts);
  }

  @Override
  public void updateLeftPID(double kP, double kI, double kD, double kV) {
    leftMotorConfigs.Slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV);

    motorLeft
        .getConfigurator()
        .apply(
            leftMotorConfigs.MotorOutput.withInverted(
                Constants.ShooterConstants.kLeftInvertedValue));
  }

  @Override
  public void updateRightPID(double kP, double kI, double kD, double kV) {
    rightMotorConfigs.Slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKV(kV);

    motorRight
        .getConfigurator()
        .apply(
            rightMotorConfigs.MotorOutput.withInverted(
                Constants.ShooterConstants.kRightInvertedValue));
  }

  @Override
  public void setVelocity(double rpm) {
    if (rpm == 0.0) {
      motorLeft.setVoltage(0.0);
      motorRight.setVoltage(0.0);
      return;
    }

    motorRight.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(rpm / 60.0);
          case TorqueCurrentFOC -> velocityTorqueCurrentFOC.withVelocity(rpm / 60.0);
        });

    motorLeft.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(rpm / 60.0);
          case TorqueCurrentFOC -> velocityTorqueCurrentFOC.withVelocity(rpm / 60.0);
        });
  }

  @Override
  public double getSetpoint() {
    return motorLeft.getClosedLoopReference().getValueAsDouble();
  }
}
