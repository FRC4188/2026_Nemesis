package frc.robot.CSPLib.pathgen;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PG_math {
  public static short clamp(short min, short val, short max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
  }

  public static float clamp(float min, float val, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
  }

  public static float dot_f(Translation2d t1, Translation2d t2) {
    return (float) (t1.getX() * t2.getX() + t1.getY() * t2.getY());
  }

  public static float point_from_lineseg_f(
      Translation2d l1, Translation2d l2, Translation2d point) {
    Translation2d m = l2.minus(l1);
    float l_sqrd = (float) (m.getX() * m.getX() + m.getY() * m.getY());
    if (l_sqrd == 0.f) return (float) l1.getDistance(point);

    float t = clamp(0, dot_f(point.minus(l1), m) / l_sqrd, 1);
    Translation2d proj = l1.plus(m.times(t));
    return (float) point.getDistance(proj);
  }

  public static float lineseg_distance_lineseg(
      Translation2d l11, Translation2d l12, Translation2d l21, Translation2d l22) {
    float result = Float.MAX_VALUE;

    float[] vals = {
      point_from_lineseg_f(l11, l12, l21),
      point_from_lineseg_f(l11, l12, l22),
      point_from_lineseg_f(l21, l22, l11),
      point_from_lineseg_f(l21, l22, l12)
    };

    for (float val : vals) {
      if (val < result) {
        result = val;
      }
    }

    return result;
  }

  private static boolean ccw(Translation2d a, Translation2d b, Translation2d c) {
    return (c.getY() - a.getY()) * (b.getX() - a.getX())
        > (b.getY() - a.getY()) * (c.getX() - a.getX());
  }
  ;

  public static boolean intersect_lineseg(
      Translation2d l11, Translation2d l12, Translation2d l21, Translation2d l22) {
    return ccw(l11, l21, l22) != ccw(l12, l21, l22) && ccw(l11, l12, l21) != ccw(l11, l12, l22);
  }

  public static Rotation2d interpolate_mod(Rotation2d orig, Rotation2d goal, double t) {
    double orig_mod = modulate(orig).getDegrees();
    double goal_mod = modulate(goal).getDegrees();

    if (goal_mod - orig_mod > 180) {
      goal_mod -= 360;
    } else if (goal_mod - orig_mod < -180) {
      orig_mod -= 360;
    }

    double result = MathUtil.clamp(t, 0, 1) * (goal_mod - orig_mod) + orig_mod;

    return Rotation2d.fromDegrees(result);
  }

  public static Rotation2d modulate(Rotation2d r) {
    return Rotation2d.fromDegrees((r.getDegrees() + 180) % 360 - 180);
  }

  public static void printpose(Translation2d t) {
    System.out.println("(" + t.getX() + ", " + t.getY() + ")");
  }

  public static void printpose(Pose2d p) {
    System.out.println("(" + p.getX() + ", " + p.getY() + ")");
  }

  public static void printpose(Translation2d... t) {
    for (Translation2d t2 : t) {
      printpose(t2);
    }
  }

  public static void printpose(Pose2d... t) {
    for (Pose2d t2 : t) {
      printpose(t2);
    }
  }
}
