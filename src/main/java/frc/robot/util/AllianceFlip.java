package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlip {

  public static double flipX(double x) {
    if (canFlip()) {
      return FieldConstants.field_length - x;
    } else {
      return x;
    }
  }

  public static double flipY(double y) {
    if (canFlip()) {
      return FieldConstants.field_width - y;
    } else {
      return y;
    }
  }

  public static Translation2d flipX(Translation2d pos) {
    return new Translation2d(FieldConstants.field_length - pos.getX(), pos.getY());
  }

  public static Pose2d flipX(Pose2d pos) {
    return new Pose2d(FieldConstants.field_length - pos.getX(), pos.getY(), pos.getRotation());
  }

  public static Translation2d flipY(Translation2d pos) {
    return new Translation2d(pos.getX(), FieldConstants.field_width - pos.getY());
  }

  public static Pose2d flipY(Pose2d pos) {
    return new Pose2d(pos.getX(), FieldConstants.field_width - pos.getY(), pos.getRotation());
  }

  public static Pose2d apply(Pose2d pos) {
    if (canFlip()) {
      return new Pose2d(flipX(pos.getX()), flipY(pos.getY()), apply(pos.getRotation()));
    } else {
      return pos;
    }
  }

  public static Translation2d apply(Translation2d pos) {
    if (canFlip()) {
      return new Translation2d(flipX(pos.getX()), flipY(pos.getY()));
    } else {
      return pos;
    }
  }

  public static Rotation2d apply(Rotation2d rotation) {
    if (canFlip()) {
      return new Rotation2d(-rotation.getCos(), -rotation.getSin());
    } else {
      return rotation;
    }
  }

  public static boolean canFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
