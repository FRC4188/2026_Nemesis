package frc.robot.subsystems.Launcher.Shooter;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  private final StatusSignal<Voltage> applied_volts_left;
  private final StatusSignal<Voltage> applied_volts_right;
  private final StatusSignal<Temperature> tempCL;
  private final StatusSignal<Temperature> tempCR;

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
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.applied_volts_left = applied_volts_left.getValueAsDouble();
    inputs.applied_volts_right = applied_volts_right.getValueAsDouble();
    inputs.tempCL = tempCL.getValueAsDouble();
    inputs.tempCR = tempCR.getValueAsDouble();
  }

  // need to add pid at some point!!!!!!!!! (use torque based Pid)
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
}
