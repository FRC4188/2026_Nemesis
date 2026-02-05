package frc.robot.subsystems.Launcher.Shooter;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  private final StatusSignal<Voltage> applied_volts_left;
  private final StatusSignal<Voltage> applied_volts_right;
  private final StatusSignal<Temperature> tempCL;
  private final StatusSignal<Temperature> tempCR;

  // add the required variables for Torque/velocity PID
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  TalonFXConfiguration configsR = new TalonFXConfiguration();
  TalonFXConfiguration configsL = new TalonFXConfiguration();

  public ShooterIOReal() {
    motorLeft = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
    motorRight = new TalonFX(Constants.Id.kRightShooter, Constants.Robot.rio);

    motorLeft.setNeutralMode(NeutralModeValue.Brake);
    motorRight.setNeutralMode(NeutralModeValue.Brake);

    applied_volts_left = motorLeft.getMotorVoltage();
    applied_volts_right = motorRight.getMotorVoltage();
    tempCL = motorLeft.getDeviceTemp();
    tempCR = motorRight.getDeviceTemp();

    applied_volts_left.setUpdateFrequency(Hertz.of(50));
    applied_volts_right.setUpdateFrequency(Hertz.of(50));
    tempCL.setUpdateFrequency(Hertz.of(0.5));
    tempCR.setUpdateFrequency(Hertz.of(0.5));

    motorLeft.optimizeBusUtilization();
    motorRight.optimizeBusUtilization();

    motorRight.getConfigurator().apply(configsR);
    motorLeft.getConfigurator().apply(configsL);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.applied_volts_left = applied_volts_left.getValueAsDouble();
    inputs.applied_volts_right = applied_volts_right.getValueAsDouble();
    inputs.tempCL = tempCL.getValueAsDouble();
    inputs.tempCR = tempCR.getValueAsDouble();
  }

  // need to add pid at some point!!!!!!!!! (use torque/velocity based Pid)
  // Will also need to implement Feed Forward as well (ughhh)
  public void runVoltsLeft(double volts) {
    motorLeft.setVoltage(volts);
  }

  public void runVoltsRight(double volts) {
    motorRight.setVoltage(volts);
  }

  public void stop() {
    motorLeft.setVoltage(0);
    motorRight.setVoltage(0);
  }

  public void updatePIDR(double kP, double kI, double kD, double kV) {
    configsR.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKV(kV);
             // IDK abt this gravity type (does it even need one?)

    PhoenixUtil.tryUntilOk(5, () -> motorRight.getConfigurator().apply(configsR, 0.25));
  }

  public void updatePIDL(double kP, double kI, double kD, double kV) {
    configsL.Slot0 =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKV(kV);
             // IDK abt this gravity type (does it even need one?)

    PhoenixUtil.tryUntilOk(5, () -> motorLeft.getConfigurator().apply(configsL, 0.25));
  }

  public void setOpenLoopRight(double outputR) {
    motorRight.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(outputR);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(outputR);
        });
  }

  public void setOpenLoopLeft(double outputL) {
    motorLeft.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(outputL);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(outputL);
        });
  }

  public double getVelocityLeft() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  public double getVelocityRight() {
    return motorRight.getVelocity().getValueAsDouble();
  }

  // These set Velocities are probably wrong, I am just trying to figure it out fr
  public void setVelocityRight(double velocity) {
    motorRight.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(velocity);
          case TorqueCurrentFOC -> velocityTorqueCurrentFOC.withVelocity(velocity);
        });
  }

  public void setVelocityLeft(double velocity) {
    motorLeft.setControl(
        switch (Constants.ShooterConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(velocity);
          case TorqueCurrentFOC -> velocityTorqueCurrentFOC.withVelocity(velocity);
        });
  }
}
