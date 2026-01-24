package frc.robot.subsystems.Loader.Wrist;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

// TODO: add PID if necessary
public class WristIOReal implements WristIO {
  private final TalonFX motor;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;
  private final StatusSignal<Voltage> appliedVolts;

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
}
