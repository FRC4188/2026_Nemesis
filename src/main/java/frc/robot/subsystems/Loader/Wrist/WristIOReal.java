package frc.robot.subsystems.Loader.Wrist;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class WristIOReal implements WristIO {
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  // private final CANcoder cancoder;
  // private final CANcoderConfiguration canConfig;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;
  // private final StatusSignal<Angle> canPos;
  private final StatusSignal<Voltage> appliedVolts;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  // private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  public WristIOReal() {
    motor = new TalonFX(Constants.Id.kWrist, Constants.Robot.rio);
    // cancoder = new CANcoder(Constants.Id.kIntakeCANCoder, Constants.Robot.rio);

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
                    .withInverted(Constants.WristConstants.kInvertedValue)) // placeholder
            .withSlot0(Constants.WristConstants.wristGains)
            .withFeedback(
                new FeedbackConfigs()
                    // .withFeedbackRemoteSensorID(Constants.Id.kIntakeCANCoder)
                    // .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    // .withRotorToSensorRatio(Constants.WristConstants.kGearRatio))
                    .withSensorToMechanismRatio(Constants.WristConstants.kGearRatio)) // No CanCoder
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

    // canConfig =
    //   new CANcoderConfiguration()
    //     .withMagnetSensor(
    //       new MagnetSensorConfigs()
    //         .withMagnetOffset(Constants.WristConstants.kCanCoderOffset)
    //         .withSensorDirection(Constants.WristConstants.kDirection));

    // cancoder.getConfigurator().apply(canConfig);

    posRots = motor.getPosition();
    tempC = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    // canPos = cancoder.getAbsolutePosition();

    appliedVolts.setUpdateFrequency(Hertz.of(50));
    posRots.setUpdateFrequency(Hertz.of(50));
    // canPos.setUpdateFrequency(50);
    tempC.setUpdateFrequency(Hertz.of(0.5));

    ParentDevice.optimizeBusUtilizationForAll(motor); // ,cancoder

    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public boolean isStalled() {
    return Math.abs(motor.getStatorCurrent().getValueAsDouble())
            > Constants.WristConstants.kStallCurrent
        && Math.abs(motor.getVelocity().getValueAsDouble()) < 0.5;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    var motorStatus = BaseStatusSignal.refreshAll(appliedVolts, tempC, posRots);
    // var encoderStatus = BaseStatusSignal.refreshAll(canPos);

    // inputs.canPos = canPos.getValueAsDouble();
    inputs.connected = motorConnectedDebouncer.calculate(motorStatus.isOK());
    // inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoderStatus.isOK());

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.posRads = Units.rotationsToRadians(posRots.getValueAsDouble());
  }

  @Override
  public void updatePID(double kP, double kI, double kD, double kG) {
    motorConfig.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKG(kG)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
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
