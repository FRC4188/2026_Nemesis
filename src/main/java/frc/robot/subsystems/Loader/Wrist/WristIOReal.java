package frc.robot.subsystems.Loader.Wrist;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PhoenixUtil;

public class WristIOReal implements WristIO {
  private final TalonFX motor;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;
  private final StatusSignal<Voltage> appliedVolts;

  // This is all for PID i think
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);

  TalonFXConfiguration configs = new TalonFXConfiguration();

  public WristIOReal() {
    motor = new TalonFX(Constants.Id.kWrist, Constants.Robot.rio);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor
        .getConfigurator()
        .apply(IntakeConstants.kMotorConfig); // this is very ugly, change this later!!!!!
    // double check this line V
    posRots = motor.getPosition();
    tempC = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    appliedVolts.setUpdateFrequency(Hertz.of(50));
    posRots.setUpdateFrequency(Hertz.of(50));
    tempC.setUpdateFrequency(Hertz.of(0.5));

    // PID motor configurator
    // optional "Motion Magic Application":
    /*
     * Configs.MotionMagic.MotionMagicAcceleration = 10;
     * Configs.MotionMagic.MotionMagicCruiseVelocity = 5; -> Not sure what this one does
     *
     */

    motor.getConfigurator().apply(configs);
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    posRots.refresh();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
    inputs.posRads = Units.rotationsToRadians(posRots.getValueAsDouble());
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
                GravityTypeValue
                    .Arm_Cosine); // not sure about this "Arm_Cosign value, might want to double
    // check that later"
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

  // override might be needed, but its erroring
  public void setOpenLoop(double output) {
    motor.setControl(
        switch (Constants.WristConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  public double getSetpoint() {
    return motor.getClosedLoopReference().getValueAsDouble();
  }
}
