package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Lightweight hood simulation with position motion and electrical telemetry. */
public class HoodIOSim implements HoodIO {
  private enum ControlMode {
    VOLTAGE,
    CURRENT,
    POSITION
  }

  private static final double LOOP_PERIOD_SECONDS = Constants.Robot.loopPeriodSecs;
  private static final double MAX_SPEED_DEGREES_PER_SECOND = 180.0;
  private static final double POSITION_KP_VOLTS_PER_DEGREE = 0.45;

  private ControlMode controlMode = ControlMode.VOLTAGE;
  private double commandedVolts = 0.0;
  private double commandedCurrentAmps = 0.0;
  private Rotation2d position = Rotation2d.kZero;
  private Rotation2d targetPosition = Rotation2d.kZero;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    appliedVolts = calculateAppliedVolts();

    double desiredSpeedDegreesPerSecond =
        appliedVolts / 12.0 * MAX_SPEED_DEGREES_PER_SECOND;
    double positionDeltaDegrees = desiredSpeedDegreesPerSecond * LOOP_PERIOD_SECONDS;

    if (controlMode == ControlMode.POSITION) {
      double errorDegrees = targetPosition.minus(position).getDegrees();
      positionDeltaDegrees =
          Math.copySign(
              Math.min(Math.abs(positionDeltaDegrees), Math.abs(errorDegrees)), errorDegrees);
    }

    double nextPositionDegrees =
        MathUtil.clamp(
            position.getDegrees() + positionDeltaDegrees,
            Constants.HoodConstants.Min_A.getDegrees(),
            Constants.HoodConstants.Max_A.getDegrees());
    position = Rotation2d.fromDegrees(nextPositionDegrees);

    currentAmps =
        controlMode == ControlMode.CURRENT
            ? Math.abs(commandedCurrentAmps)
            : Math.min(
                Constants.HoodConstants.kStatorCurrent,
                Math.abs(appliedVolts) / 12.0 * Constants.HoodConstants.kStatorCurrent);

    inputs.motorConnected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    inputs.position = position;
  }

  @Override
  public void setVolts(double output) {
    controlMode = ControlMode.VOLTAGE;
    commandedVolts = MathUtil.clamp(output, -12.0, 12.0);
  }

  @Override
  public void setCurrent(double output) {
    controlMode = ControlMode.CURRENT;
    commandedCurrentAmps =
        MathUtil.clamp(
            output,
            Constants.HoodConstants.kPeakReverseTC,
            Constants.HoodConstants.kPeakForwardTC);
  }

  @Override
  public void setPosition(Rotation2d targetPosition) {
    controlMode = ControlMode.POSITION;
    this.targetPosition = targetPosition;
  }

  @Override
  public void setZero() {
    position = Rotation2d.kZero;
    targetPosition = Rotation2d.kZero;
  }

  private double calculateAppliedVolts() {
    return switch (controlMode) {
      case VOLTAGE -> commandedVolts;
      case CURRENT -> MathUtil.clamp(
          commandedCurrentAmps / Constants.HoodConstants.kStatorCurrent * 12.0, -12.0, 12.0);
      case POSITION -> MathUtil.clamp(
          targetPosition.minus(position).getDegrees() * POSITION_KP_VOLTS_PER_DEGREE,
          -12.0,
          12.0);
    };
  }
}
