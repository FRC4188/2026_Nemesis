package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ShotCalc {

  /** Maximum shooter speed used by the calculator. */
  public static final double kMaxRPM = 5000.0;

  /** Reference velocity for the ballistic model, in m/s. */
  private static final double kReferenceVelocityMPS = 6.435;

  /** Reference horizontal distance, in meters. */
  private static final double kReferenceDistanceMeters = 1.7716;

  /** Air density, in kg/m^3. */
  private static final double kAirDensity = 0.0023769;

  /** Ball drag coefficient. */
  private static final double kDragCoefficient = 0.47;

  /** Ball cross-sectional area, in m^2. */
  private static final double kBallArea = 0.0295;

  /** Mass of the ball, in kg. */
  private static final double kBallMass = 0.0147;


  public static double factorEstimatedDrag(double velocityMPS) {
    final double dragFactor = (kAirDensity * kDragCoefficient * kBallArea) / (2.0 * kBallMass);

    return velocityMPS / (1.0 - dragFactor);
  }

  public static double velocityMPSToRPM(double velocityMPS) {
    return 0.0; // Placeholder for actual conversion logic
  }

  public static double getShotMPS(double distanceMeters) {
    return Math.sqrt(Math.pow(6.434946806, 2) + Math.pow(distanceMeters / 1.03702895, 2));
  }

  public static Rotation2d getShotAngle(double distanceMeters) {
    return Rotation2d.fromDegrees(90 - Math.toDegrees(Math.atan(6.434946806 * 1.03702895 / distanceMeters)));
  }
}
