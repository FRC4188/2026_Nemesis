package frc.robot.subsystems.Launcher.Hood;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class HoodIOReal implements HoodIO {
  private final TalonFX motor;
  private final CANcoder canCoder;

  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Angle> angleRads;

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
  //   @Override
  //   public double getSetpoint(){
  //     return
  //   }
}
