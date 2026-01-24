package frc.robot.subsystems.Launcher.Shooter;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX motor1;
  private final TalonFX motor2;

  private final StatusSignal<Voltage> applied_volts_left;
  private final StatusSignal<Voltage> applied_volts_right;
  private final StatusSignal<Temperature> tempCL;
  private final StatusSignal<Temperature> tempCR;

  public ShooterIOReal() {
    motor1 = new TalonFX(Constants.Id.kLeftShooter, Constants.Robot.rio);
    motor2 = new TalonFX(Constants.Id.kRightShooter, Constants.Robot.rio);

    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);

    applied_volts_left = motor1.getMotorVoltage();
    applied_volts_right = motor2.getMotorVoltage();
    tempCL = motor1.getDeviceTemp();
    tempCR = motor2.getDeviceTemp();

    applied_volts_left.setUpdateFrequency(Hertz.of(50));
    applied_volts_right.setUpdateFrequency(Hertz.of(50));
    tempCL.setUpdateFrequency(Hertz.of(0.5));
    tempCR.setUpdateFrequency(Hertz.of(0.5));

    motor1.optimizeBusUtilization();
    motor2.optimizeBusUtilization();
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.applied_volts_left = applied_volts_left.getValueAsDouble();
    inputs.applied_volts_right = applied_volts_right.getValueAsDouble();
    inputs.tempC1 = tempCL.getValueAsDouble();
    inputs.tempC2 = tempCR.getValueAsDouble();
  }

  // need to add pid at some point!!!!!!!!! (use torque based Pid)
  public void runVoltsLeft(double volts) {
    motor1.setVoltage(volts);
  }

  public void runVoltsRight(double volts) {
    motor2.setVoltage(volts);
  }

  public void stop() {
    motor1.setVoltage(0);
    motor2.setVoltage(0);
  }
}
