package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/** Lightweight shooter simulation with closed-loop velocity and electrical telemetry. */
public class ShooterIOSim implements ShooterIO {
  private enum ControlMode {
    VOLTAGE,
    TORQUE_CURRENT,
    VELOCITY
  }

  private static final double LOOP_PERIOD_SECONDS = Constants.Robot.loopPeriodSecs;
  private static final double RESPONSE_TIME_SECONDS = 0.18;
  private static final double VELOCITY_KP_VOLTS_PER_RPM = 0.004;

  private ControlMode controlMode = ControlMode.TORQUE_CURRENT;
  private double commandedVolts = 0.0;
  private double commandedCurrentAmps = 0.0;
  private double targetRPM = 0.0;
  private double leftRPM = 0.0;
  private double rightRPM = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    appliedVolts = calculateAppliedVolts();

    double freeSpeedRPM =
        appliedVolts / 12.0 * Constants.ShooterConstants.kMaxRPM;
    double responseFraction = 1.0 - Math.exp(-LOOP_PERIOD_SECONDS / RESPONSE_TIME_SECONDS);
    leftRPM += (freeSpeedRPM - leftRPM) * responseFraction;
    rightRPM += (freeSpeedRPM - rightRPM) * responseFraction;

    double averageRPM = (leftRPM + rightRPM) / 2.0;
    double backEmfVolts =
        averageRPM / Constants.ShooterConstants.kMaxRPM * 12.0;
    currentAmps =
        controlMode == ControlMode.TORQUE_CURRENT
            ? Math.abs(commandedCurrentAmps)
            : Math.min(
                Constants.ShooterConstants.kStatorCurrent,
                Math.abs(appliedVolts - backEmfVolts)
                    / 12.0
                    * Constants.ShooterConstants.kStatorCurrent);

    inputs.leftConnected = true;
    inputs.rightConnected = true;
    inputs.left2Connected = true;
    inputs.left3Connected = true;

    inputs.leftAppliedVolts = appliedVolts;
    inputs.rightAppliedVolts = appliedVolts;
    inputs.left2AppliedVolts = appliedVolts;
    inputs.left3AppliedVolts = appliedVolts;

    inputs.leftCurrentAmps = currentAmps;
    inputs.rightCurrentAmps = currentAmps;
    inputs.left2CurrentAmps = currentAmps;
    inputs.left3CurrentAmps = currentAmps;

    inputs.leftVelocityRPM = leftRPM;
    inputs.rightVelocityRPM = rightRPM;
    inputs.left2VelocityRPM = leftRPM;
    inputs.left3VelocityRPM = leftRPM;
  }

  @Override
  public void setVolts(double volts) {
    controlMode = ControlMode.VOLTAGE;
    commandedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setTorqueCurrent(double amps) {
    controlMode = ControlMode.TORQUE_CURRENT;
    commandedCurrentAmps =
        MathUtil.clamp(
            amps,
            Constants.ShooterConstants.kPeakReverseTC,
            Constants.ShooterConstants.kPeakForwardTC);
  }

  @Override
  public void setVelocity(double rpm) {
    controlMode = ControlMode.VELOCITY;
    targetRPM = MathUtil.clamp(rpm, 0.0, Constants.ShooterConstants.kMaxRPM);
  }

  private double calculateAppliedVolts() {
    return switch (controlMode) {
      case VOLTAGE -> commandedVolts;
      case TORQUE_CURRENT -> MathUtil.clamp(
          commandedCurrentAmps / Constants.ShooterConstants.kStatorCurrent * 12.0, -12.0, 12.0);
      case VELOCITY -> {
        double averageRPM = (leftRPM + rightRPM) / 2.0;
        double feedforwardVolts =
            targetRPM / Constants.ShooterConstants.kMaxRPM * 12.0;
        double feedbackVolts = (targetRPM - averageRPM) * VELOCITY_KP_VOLTS_PER_RPM;
        yield MathUtil.clamp(feedforwardVolts + feedbackVolts, -12.0, 12.0);
      }
    };
  }
}
