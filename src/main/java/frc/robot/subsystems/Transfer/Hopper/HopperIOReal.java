package frc.robot.subsystems.Transfer.Hopper;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class HopperIOReal implements HopperIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempC;

  public HopperIOReal() {
    motor = new TalonFX(Constants.Id.kHopper, Constants.Robot.rio);

    motor.setNeutralMode(NeutralModeValue.Brake);
    motor
        .getConfigurator()
        .apply(
            IntakeConstants
                .kMotorConfig); // for now this is set to the same config as all of the other
    // classes
    appliedVolts = motor.getMotorVoltage();
    tempC = motor.getDeviceTemp();

    appliedVolts.setUpdateFrequency(Hertz.of(50));
    tempC.setUpdateFrequency(Hertz.of(0.5));

    motor.optimizeBusUtilization();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
  }
}
