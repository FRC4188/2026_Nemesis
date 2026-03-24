package frc.robot.CSPLib.inputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CSP_Controller extends CommandXboxController {
  public enum Scale {
    LINEAR,
    SQUARED,
    CUBED,
    QUARTIC
  }

  public CSP_Controller(int port) {
    super(port);
  }

  private static double getOutput(double input, Scale scale) {
    return scaleValue(MathUtil.applyDeadband(input, Constants.Controller.DEADBAND), scale);
  }

  private static double scaleValue(double input, Scale scale) {
    switch (scale) {
      case LINEAR:
        return input;
      case SQUARED:
        return Math.signum(input) * Math.pow(input, 2);
      case CUBED:
        return Math.pow(input, 3);
      case QUARTIC:
        return Math.signum(input) * Math.pow(input, 4);
      default:
        return input;
    }
  }

  public Translation2d getCorrectedRight(Scale scale) {
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(super.getRightX(), super.getRightY()), Constants.Controller.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(super.getRightY(), super.getRightX()));

    linearMagnitude = scaleValue(linearMagnitude, scale);

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public Translation2d getCorrectedLeft(Scale scale) {
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(super.getLeftX(), super.getLeftY()), Constants.Controller.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(super.getLeftY(), super.getLeftX()));

    linearMagnitude = scaleValue(linearMagnitude, scale);

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public double getLeftY(Scale scale) {
    return getOutput(getLeftY(), scale);
  }

  public double getLeftX(Scale scale) {
    return getOutput(getLeftX(), scale);
  }

  public double getRightY(Scale scale) {
    return getOutput(getRightY(), scale);
  }

  public double getRightX(Scale scale) {
    return getOutput(getRightX(), scale);
  }

  public double getRightT(Scale scale) {
    return getOutput(getRightTriggerAxis(), scale);
  }

  public double getLeftT(Scale scale) {
    return getOutput(getLeftTriggerAxis(), scale);
  }

  public Trigger getRightTButton() {
    return this.rightTrigger(0.1);
  }

  public Trigger getLeftTButton() {
    return this.leftTrigger(0.1);
  }
}
