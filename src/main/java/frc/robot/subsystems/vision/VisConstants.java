package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisConstants {
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static String frontPho = "front";
  public static String backPho = "back";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.14, 0.343 - Units.inchesToMeters(16), 0.356, new Rotation3d(0.0, 0, 0));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          0.14 - 2 * Units.inchesToMeters(0.42),
          0.343 - Units.inchesToMeters(18) - 2 * Units.inchesToMeters(1.125),
          0.356 + Units.inchesToMeters(0.5),
          new Rotation3d(0.0, 0.0, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;
  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters 0.02 default
  public static double angularStdDevBaseline = 0.06; // Radians 0.06 default

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static int algaeDetect = 0; // placeholder
  public static int aprilTagDetect = 1; // placeholder
  public static double left_region = -10.0; // left region of cam (left side reef)
  public static double right_region = 10.0; // right region of cam (right side reef)
}
