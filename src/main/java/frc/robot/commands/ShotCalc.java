package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ShotCalc {

  /** Maximum shooter speed used by the calculator. */
  public static final double kMaxRPM = 5000.0;

  /** Air density, in kg/m^3. */
  private static final double kAirDensity = 0.0023769;

  /** Ball drag coefficient. */
  private static final double kDragCoefficient = 0.47;

  /** Ball cross-sectional area, in m^2. */
  private static final double kBallArea = 0.0295;

  /** Mass of the ball, in kg. */
  private static final double kBallMass = 0.0147;

  // theoretical to actual (more to less)
  public static double factorEstimatedDrag(double velocityMPS) {
    final double dragFactor = (kAirDensity * kDragCoefficient * kBallArea) / (2.0 * kBallMass);

    return velocityMPS / (1.0 - dragFactor);
  }

  // actual to theoretical (less to more)
  public static double inverseFactorEstimatedDrag(double velocityMPS) {
    final double dragFactor = (kAirDensity * kDragCoefficient * kBallArea) / (2.0 * kBallMass);

    return velocityMPS * (1.0 - dragFactor);
  }

  public static double getShotMPS(double distanceMeters) {
    return Math.sqrt(Math.pow(6.434946806, 2) + Math.pow(distanceMeters / 1.03702895, 2));
  }

  public static Rotation2d getShotAngle(double distanceMeters) {
    return Rotation2d.fromDegrees(
        90 - Math.toDegrees(Math.atan(6.434946806 * 1.03702895 / distanceMeters)));
  }

  public static double velocityMPSToRPM(double mps) {
    return 302.1148 * mps - 181.26888;
  }

  public static double getShotRPM(double distanceMeters) {
    return velocityMPSToRPM(inverseFactorEstimatedDrag(getShotMPS(distanceMeters)));
  }
}
