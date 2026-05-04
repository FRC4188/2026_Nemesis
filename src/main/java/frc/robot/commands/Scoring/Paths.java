package frc.robot.commands.Scoring;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.pathbuilder.PathBuilder;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;

public class Paths {
  public static PathPlannerPath firstSwipe =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(4.515, 0.501, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(3.239))
              .withStartingRotation(Rotation2d.fromDegrees(89.326))
              .withStartingSpeed(2)
              .withOverrideRotations(new RotationTarget(1.29, Rotation2d.fromDegrees(90)))
              .withControlDistances(0, 1.158),
          new PathBuilder.Target(new Pose2d(6.538, 0.630, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(24.520))
              .withControlDistances(0.548, 0.992)
              .withSpeed(0.2),
          new PathBuilder.Target(new Pose2d(7.958, 2.028, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(66.991))
              .withControlDistances(0.250, 0.250)
              .withSpeed(0.2),
          new PathBuilder.Target(new Pose2d(7.850, 3.320, Rotation2d.fromDegrees(103.092)))
              .withHeading(Rotation2d.fromDegrees(85.207))
              .withControlDistances(0.250, 0)
              .withEndingSpeed(0)
              .withEndingRotation(Rotation2d.fromDegrees(103.092))
              .withSpeed(0.2));

  public static PathPlannerPath followerFirstSwipe =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(3.515, 0.501, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(3.239))
              .withStartingRotation(Rotation2d.fromDegrees(89.326))
              .withStartingSpeed(2)
              .withOverrideRotations(new RotationTarget(1.29, Rotation2d.fromDegrees(90)))
              .withControlDistances(0, 1.158),
          new PathBuilder.Target(new Pose2d(6.538, 0.630, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(24.520))
              .withControlDistances(0.548, 0.992)
              .withSpeed(0.2),
          new PathBuilder.Target(new Pose2d(7.958, 2.028, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(66.991))
              .withControlDistances(0.250, 0.250)
              .withSpeed(0.2),
          new PathBuilder.Target(new Pose2d(7.850, 3.320, Rotation2d.fromDegrees(103.092)))
              .withHeading(Rotation2d.fromDegrees(85.207))
              .withControlDistances(0.250, 0)
              .withEndingSpeed(0)
              .withEndingRotation(Rotation2d.fromDegrees(103.092))
              .withSpeed(0.2));

  public static PathPlannerPath trenchReturn =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(7.850, 3.320, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-123.951))
              .withControlDistances(0, 0.989)
              .withStartingRotation(Rotation2d.fromDegrees(103.092))
              .withStartingSpeed(2)
              .withOverrideRotations(new RotationTarget(1.00, Rotation2d.fromDegrees(91.150))),
          new PathBuilder.Target(new Pose2d(6.486, 0.598, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-157.306))
              .withControlDistances(1.172, 0.601),
          new PathBuilder.Target(new Pose2d(4.545, 0.427, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(178.986))
              .withControlDistances(0.686, 0.589),
          new PathBuilder.Target(new Pose2d(2.878, 0.639, Rotation2d.fromDegrees(90)))
              .withHeading(Rotation2d.fromDegrees(-178.975))
              .withControlDistances(0.544, 0)
              .withEndingRotation(Rotation2d.fromDegrees(103.092))
              .withEndingSpeed(0));

  public static PathPlannerPath bumpReturn =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(7.850, 3.320, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-164.215))
              .withControlDistances(0, 2.307)
              .withStartingSpeed(2)
              .withStartingRotation(Rotation2d.fromDegrees(103.092))
              .withOverrideRotations(
                  new RotationTarget(0.77, Rotation2d.kCCW_90deg),
                  new RotationTarget(1.22, Rotation2d.kCCW_90deg)),
          new PathBuilder.Target(new Pose2d(4.644, 2.373, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(176.561))
              .withControlDistances(1.894, 2.511),
          new PathBuilder.Target(new Pose2d(1.040, 0.737, Rotation2d.fromDegrees(43.199)))
              .withHeading(Rotation2d.fromDegrees(-162.033))
              .withControlDistances(0.344, 0)
              .withEndingSpeed(0)
              .withEndingRotation(Rotation2d.fromDegrees(43.199)));

  public static PathPlannerPath trenchToSecond =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(2.878, 0.639, Rotation2d.fromDegrees(70.846)))
              .withHeading(Rotation2d.kCCW_90deg)
              .withControlDistances(0, 0.250)
              .withStartingSpeed(0)
              .withStartingRotation(Rotation2d.fromDegrees(70.846))
              .withSpeed(0.444)
              .withOverrideRotations(new RotationTarget[0]),
          new PathBuilder.Target(new Pose2d(4.515, 0.382, Rotation2d.kCCW_90deg))
              .withHeading(Rotation2d.fromDegrees(9.951))
              .withControlDistances(0.623, 0)
              .withEndingRotation(Rotation2d.kCCW_90deg)
              .withEndingSpeed(2)
              .withSpeed(0.444));

  public static PathPlannerPath secondSwipe =
      PathBuilder.build(
          new PathBuilder.Target(
                  new Pose2d(
                      FieldConstants.Trench.right_trench_center.plus(new Translation2d(0, -0.18)),
                      Rotation2d.kCCW_90deg))
              .withStartingSpeed(5)
              .withStartingRotation(Rotation2d.kCCW_90deg)
              .withOverrideRotations(
                  new RotationTarget(0.97, Rotation2d.fromDegrees(87.075)),
                  new RotationTarget(0.60, Rotation2d.kCCW_90deg),
                  new RotationTarget(2.00, Rotation2d.fromDegrees(110.726)),
                  new RotationTarget(3.00, Rotation2d.fromDegrees(-95.856)),
                  new RotationTarget(3.34, Rotation2d.fromDegrees(-85.402)))
              .withHeading(Rotation2d.fromDegrees(61.763))
              .withControlDistances(0, 0.250),
          new PathBuilder.Target(new Pose2d(7.355, 1.523 - 0.18, Rotation2d.kZero))
              .withHeading(Rotation2d.fromDegrees(66.360))
              .withControlDistances(1.517, 0.476),
          new PathBuilder.Target(new Pose2d(7.614, 3.051, Rotation2d.kZero))
              .withHeading(Rotation2d.fromDegrees(120.689))
              .withControlDistances(0.288, 1.250),
          new PathBuilder.Target(new Pose2d(5.968, 3.051, Rotation2d.kZero))
              .withHeading(Rotation2d.fromDegrees(-104.349))
              .withControlDistances(0.955, 0.310),
          new PathBuilder.Target(new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
              .withHeading(Rotation2d.fromDegrees(99.792))
              .withControlDistances(0.250, 0)
              .withEndingRotation(Rotation2d.kZero)
              .withEndingSpeed(2));

  public static PathPlannerPath secondToTrench =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
              .withStartingSpeed(2),
          new PathBuilder.Target(
              new Pose2d(
                  FieldConstants.Trench.right_trench_center.plus(new Translation2d(0, -0.15)),
                  Rotation2d.kZero)),
          new PathBuilder.Target(
              new Pose2d(
                  FieldConstants.Trench.right_trench_center.plus(new Translation2d(-2.2, 0.4)),
                  Rotation2d.kZero)));

  public static PathPlannerPath bumpReturnSwipe =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(7.850, 3.320, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-137.466))
              .withControlDistances(0, 1.202)
              .withStartingSpeed(0)
              .withStartingRotation(Rotation2d.fromDegrees(103.092))
              .withOverrideRotations(
                  new RotationTarget(1, Rotation2d.kZero),
                  new RotationTarget(2.15, Rotation2d.kZero)),
          new PathBuilder.Target(new Pose2d(5.660, 2.382, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-179.842))
              .withControlDistances(0.686, 1.812),
          new PathBuilder.Target(new Pose2d(3.493, 2.382, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-179.617))
              .withControlDistances(2.144, 0.301),
          new PathBuilder.Target(new Pose2d(2.445, 1.909, Rotation2d.fromDegrees(40.495)))
              .withHeading(Rotation2d.fromDegrees(-143.467))
              .withControlDistances(0.395, 0)
              .withEndingRotation(Rotation2d.fromDegrees(40.495))
              .withEndingSpeed(0));

  public static PathPlannerPath bumpToTrench =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(2.445, 1.909, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-115.699))
              .withControlDistances(0, 1.789)
              .withStartingSpeed(0)
              .withStartingRotation(Rotation2d.fromDegrees(40.495))
              .withOverrideRotations(new RotationTarget(0.35, Rotation2d.fromDegrees(85.580))),
          new PathBuilder.Target(new Pose2d(4.632, 0.457, Rotation2d.fromDegrees(90)))
              .withHeading(Rotation2d.fromDegrees(-4.357))
              .withControlDistances(1.061, 0)
              .withEndingRotation(Rotation2d.fromDegrees(90))
              .withEndingSpeed(3.5));

  public static PathPlannerPath hairpinBump =
      PathBuilder.build(
          new PathBuilder.Target(new Pose2d(4.632, 0.457, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-57.768))
              .withControlDistances(0, 0.261)
              .withStartingSpeed(5)
              .withStartingRotation(Rotation2d.fromDegrees(90))
              .withOverrideRotations(
                  new RotationTarget(0.60, Rotation2d.fromDegrees(89.254)),
                  new RotationTarget(0.97, Rotation2d.fromDegrees(87.075)),
                  new RotationTarget(2, Rotation2d.fromDegrees(110.726)),
                  new RotationTarget(2.71, Rotation2d.fromDegrees(-134.120)),
                  new RotationTarget(3, Rotation2d.fromDegrees(1.332)),
                  new RotationTarget(3.34, Rotation2d.fromDegrees(1.040))),
          new PathBuilder.Target(new Pose2d(7.494, 1.274, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(50.285))
              .withControlDistances(1.384, 0.427),
          new PathBuilder.Target(new Pose2d(7.847, 3.118, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(127.772))
              .withControlDistances(0.330, 1.139),
          new PathBuilder.Target(new Pose2d(6.154, 2.533, new Rotation2d()))
              .withHeading(Rotation2d.fromDegrees(-137.976))
              .withControlDistances(0.676, 0.250),
          new PathBuilder.Target(new Pose2d(2.445, 1.909, Rotation2d.fromDegrees(40.495)))
              .withHeading(Rotation2d.fromDegrees(-128.100))
              .withControlDistances(1.127, 0)
              .withEndingRotation(Rotation2d.fromDegrees(40.495))
              .withEndingSpeed(0));

  public static Pose2d getFirstPose(PathPlannerPath path) {
    return AllianceFlip.apply(
        (path.getStartingHolonomicPose().isEmpty())
            ? path.getStartingHolonomicPose().get()
            : new Pose2d(
                path.getWaypoints().get(0).anchor(), path.getRotationTargets().get(0).rotation()));
  }
}
