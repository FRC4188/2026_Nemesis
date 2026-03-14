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

  public static Pose2d apply(Pose2d pos) {
    if (canFlip()) {
      return new Pose2d(flipX(pos.getX()), flipY(pos.getY()), apply(pos.getRotation()));
    } else {
      return pos;
    }
  }

  // public static List<Pose2d> apply(List<Pose2d> poses) {
  //   List<Pose2d> temp = new ArrayList<>();

  //   for (Pose2d pos : poses.toArray(new Pose2d[0])) {
  //     if (canFlip()) {
  //       temp.add(new Pose2d(flipX(pos.getX()), flipY(pos.getY()), apply(pos.getRotation())));
  //     } else {
  //       temp.add(pos);
  //     }
  //   }

  //   return temp;
  // }

  // public static List<Translation2d> apply(List<Translation2d> poses) {
  //   List<Translation2d> temp = new ArrayList<>();

  //   for (Translation2d pos : poses.toArray(new Translation2d[0])) {
  //     if (canFlip()) {
  //       temp.add(new Translation2d(flipX(pos.getX()), flipY(pos.getY()))));
  //     } else {
  //       temp.add(pos);
  //     }
  //   }

  //   return temp;
  // }

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
