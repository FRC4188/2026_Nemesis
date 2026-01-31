package frc.robot.subsystems.Launcher.Hood;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PhoenixUtil;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;
  private final CANcoder canCoder;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Angle> angleRads;

  // this is all for PID
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  TalonFXConfiguration configs = new TalonFXConfiguration();

  public HoodIOReal() {
    motor = new TalonFX(Constants.Id.kHood, Constants.Robot.rio);
    canCoder = new CANcoder(Constants.Id.kHoodCANCoder); // add this to constants

    motor.setNeutralMode(NeutralModeValue.Brake);
    motor
        .getConfigurator()
        .apply(
            IntakeConstants
                .kMotorConfig); // change intakeconstants to hoodconstants once motor config is
    // added to hoodconstants

    tempC = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    angleRads = canCoder.getAbsolutePosition();

    tempC.setUpdateFrequency(Hertz.of(0.5));
    appliedVolts.setUpdateFrequency(Hertz.of(50));

    motor.optimizeBusUtilization();

    // optional "Motion Magic Application":
    /*
     * Configs.MotionMagic.MotionMagicAcceleration = 10;
     * Configs.MotionMagic.MotionMagicCruseVelocity = 5; -> Not sure what this is
     */

    configs.Feedback.SensorToMechanismRatio =
        3.0; // allegedly setting the sensor to mechanism ratio correctly 3:1

    motor.getConfigurator().apply(configs);
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
  }

  @Override
  public double getAngle() {
    return Units.rotationsToRadians(angleRads.getValueAsDouble());
  }

  @Override
  public void updatePID(double kP, double kI, double kD, double kG) {
    configs.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKG(kG)
            .withGravityType(
                GravityTypeValue.Arm_Cosine); // not sure if this is the correct gravity value
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(configs, 0.25));
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

  public void setOpenLoop(double output) {
    motor.setControl(
        switch (Constants.HoodConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  public double getSetpoint() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }
}
