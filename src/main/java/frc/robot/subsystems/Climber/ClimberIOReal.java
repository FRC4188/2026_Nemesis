package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Temperature> tempC;
  private final StatusSignal<Angle> posRots;

  private final VoltageOut voltReq = new VoltageOut(0);
  private final VoltageOut voltFOC = new VoltageOut(0).withEnableFOC(true);

  public ClimberIOReal() {
    motor = new TalonFX(Constants.Id.kClimber, Constants.Robot.rio);

    motor.clearStickyFaults();
    motor
        .getConfigurator()
        .apply(Constants.IntakeConstants.kMotorConfig); // change this, it is ugly
    motor.optimizeBusUtilization();

    posRots = motor.getPosition();
    appliedVolts = motor.getMotorVoltage();
    tempC = motor.getDeviceTemp();
    posRots.setUpdateFrequency(Hertz.of(50));
    appliedVolts.setUpdateFrequency(Hertz.of(50));
    tempC.setUpdateFrequency(Hertz.of(0.5));
  }

  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(posRots, appliedVolts, tempC).isOK();

    inputs.posRads = Units.rotationsToRadians(posRots.getValueAsDouble());
    inputs.tempC = tempC.getValueAsDouble();
    inputs.applied_volts = appliedVolts.getValueAsDouble();
  }

  public void runVolts(double volts) {
    MathUtil.clamp(volts, -12, 12);
    if (ClimberConstants.isFOC) {
      motor.setControl(voltReq.withOutput(volts));
    } else {
      motor.setControl(voltFOC.withOutput(volts));
    }
  }

  public double getAngle() {
    return Units.rotationsToRadians(posRots.getValueAsDouble());
  }
}
