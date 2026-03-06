package frc.robot.CSPLib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ProjMath {

  public static double grav = 9.80665;

  // Return higher incline angle of shooter based on shooter velocity and goal position
  public static Rotation2d staticShot(double velocity, Translation2d goal) {
    // goal is vertical so shoot vertical
    if (goal.getX() == 0) {
      return Rotation2d.kCW_90deg;
    }

    double boundary =
        Math.pow(velocity, 4)
            - 2 * Math.pow(velocity, 2) * goal.getY() * grav
            + Math.pow(grav, 2) * Math.pow(goal.getX(), 2);

    if (boundary < 0) return Rotation2d.kCCW_90deg;

    return Rotation2d.fromRadians(
        Math.atan((Math.pow(velocity, 2) + Math.sqrt(boundary)) / (grav * goal.getX())));
  }

  // Return rotation associated with higher shooting angle for shooting on the move
  public static Rotation3d movingShot(double shootvel, Translation3d goal, Translation2d robotvel) {
    double t = timeQuartic(shootvel, goal, robotvel);

    if (t < 0) return new Rotation3d(0, -Math.PI / 2, 0);

    return new Rotation3d(
        0,
        Math.acos(
            robotvel.minus(new Translation2d(goal.getX() * t, goal.getY() * t)).getNorm()
                / (t * shootvel)),
        Math.atan2(goal.getY() - robotvel.getY() * t, goal.getX() - robotvel.getX() * t));
  }

  // i love math
  private static double timeQuartic(double shootvel, Translation3d goal, Translation2d robotvel) {
    double a =
        (4.0 / Math.pow(grav, 2))
            * (grav * goal.getZ()
                + Math.pow(robotvel.getX(), 2)
                + Math.pow(robotvel.getY(), 2)
                - Math.pow(shootvel, 2));
    double b =
        -(8.0 / Math.pow(grav, 2))
            * (goal.getX() * robotvel.getX() + goal.getY() * robotvel.getY());
    double c = (4.0 / Math.pow(grav, 2)) * (goal.getSquaredNorm());

    double P = -Math.pow(a, 2) / 12.0 - c;
    double Q = -Math.pow(a, 3) / 108.0 + a * c / 3.0 - Math.pow(b, 2) / 8.0;
    double S = Math.pow(Q, 4) / 4.0 + Math.pow(P, 3) / 27.0;

    double Y = 0;

    if (S >= 0) {
      double U = Math.cbrt(-Q / 2.0 + Math.sqrt(S));
      Y = -5.0 / 6.0 * a + U - P / (3.0 * U);
    } else if (S == Q / 2) {
      Y = -Math.cbrt(Q);
    } else {
      double a1 = -Q / 2;
      double b1 = Math.sqrt(Math.abs(S));
      double c1 = Math.cbrt(Math.hypot(a1, b1));
      double d1 = Math.atan2(b1, a1) / 3.0;

      Y = -5.0 / 6.0 * a + (c1 - P / c1) * Math.cos(d1);
    }

    double W = Math.sqrt(a + 2 * Y);

    double boundary = -(3 * a + 2 * Y + 2 * b / W);

    if (boundary < 0) return -1;

    return (W + Math.sqrt(boundary)) / 2.0;
  }
}
