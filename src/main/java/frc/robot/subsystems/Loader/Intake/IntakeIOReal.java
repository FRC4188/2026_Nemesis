package frc.robot.subsystems.Loader.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Temperature> tempC;

  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);

  public IntakeIOReal() {
    motor = new TalonFX(Constants.Id.kIntake, Constants.Robot.rio);

    motorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.IntakeConstants.kStatorCurrent)
                    .withSupplyCurrentLimit(Constants.IntakeConstants.kSupplyCurrent)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(Constants.IntakeConstants.kNuetralMode)
                    .withInverted(Constants.IntakeConstants.kInvertedValue))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(Constants.IntakeConstants.kPeakForwardTC)
                    .withPeakReverseTorqueCurrent(Constants.IntakeConstants.kPeakReverseTC));

    motor.getConfigurator().apply(motorConfig);

    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    tempC = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(5.0, appliedVolts, tempC, currentAmps);

    motor.optimizeBusUtilization();
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(
        switch (Constants.IntakeConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public boolean isStalled() {
    return Math.abs(currentAmps.getValueAsDouble()) > IntakeConstants.kStatorCurrent
        && Math.abs(motor.getVelocity().getValueAsDouble()) < 0.2;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.connected =
        motorConnectedDebounce.calculate(
            BaseStatusSignal.refreshAll(appliedVolts, currentAmps, tempC).isOK());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempC = tempC.getValueAsDouble();
  }
}
