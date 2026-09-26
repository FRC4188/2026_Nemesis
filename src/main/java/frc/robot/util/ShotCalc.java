package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShotCalc {

  public static final double kMaxRPM = 5000.0;
  private static final double kAirDensity = 0.0023769;
  private static final double kDragCoefficient = 0.47;
  private static final double kBallArea = 0.0295;
  private static final double kBallMass = 0.0147;

  private static final double kInitialHeight = 0.4287774;
  private static final double kGravity = 9.80665;

  public static LoggedNetworkNumber CC = new LoggedNetworkNumber("Aim Tuning/CC", 1.025);

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

  public static double getTimeOfFlight(double apex, double target) {
    return (Math.sqrt(2 * kGravity * (apex - kInitialHeight))
            + Math.sqrt(2 * kGravity * (apex - target)))
        / kGravity;
  }

  public static double getShotMPS(double distanceMeters, double apex, double target) {
    return Math.sqrt(
        2 * kGravity * (apex - kInitialHeight)
            + Math.pow(distanceMeters / getTimeOfFlight(apex, target), 2));
  }

  public static Rotation2d getShotAngle(double distanceMeters, double apex, double target) {
    return Rotation2d.fromDegrees(
        90
            - Math.toDegrees(
                Math.atan(
                    Math.sqrt(2 * kGravity * (apex - kInitialHeight))
                        * getTimeOfFlight(apex, target)
                        / distanceMeters)));
  }

  public static double velocityMPSToRPM(double mps) {
    return 302.1148 * mps - 181.26888;
  }

  public static double getShotRPM(double distanceMeters, double apex, double target) {
    return velocityMPSToRPM(inverseFactorEstimatedDrag(getShotMPS(distanceMeters, apex, target)))
        * CC.getAsDouble();
  }
}
