package frc.robot.CSPLib.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CSPLib.pathgen.PG_math;

public class CircleFO extends FieldObject {
  protected float radius = 0;

  CircleFO() {}

  public CircleFO(float x, float y, float radius) {
    super(x, y);
    this.radius = radius;
  }

  @Override
  public float from_line(Translation2d l1, Translation2d l2) {
    return Math.abs(PG_math.point_from_lineseg_f(l1, l2, new Translation2d(c_x, c_y)) - radius);
  }

  @Override
  public float from_point(Translation2d point) {
    return (float)
        Math.abs(
            (c_x - point.getX()) * (c_x - point.getX())
                + (c_y - point.getY()) * (c_y - point.getY())
                - radius);
  }
}
