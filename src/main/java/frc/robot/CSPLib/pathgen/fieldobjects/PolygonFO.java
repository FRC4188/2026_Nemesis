package frc.robot.CSPLib.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CSPLib.pathgen.PG_math;

public class PolygonFO extends FieldObject {
  Translation2d[] vertices;

  public PolygonFO() {}

  public PolygonFO(boolean closed_loop, Translation2d... vertices) {
    super(-1, -1);

    if (closed_loop) {
      this.vertices = new Translation2d[vertices.length + 1];

      for (int i = 0; i < vertices.length; i++) {
        this.vertices[i] = vertices[i];
      }

      this.vertices[vertices.length] = vertices[vertices.length - 1];
    } else {
      this.vertices = vertices;
    }
  }

  @Override
  public float from_line(Translation2d l1, Translation2d l2) {
    float distance = Float.MAX_VALUE;

    for (int i = 0; i < vertices.length - 1; i++) {
      float dis = PG_math.lineseg_distance_lineseg(vertices[i], vertices[(i + 1)], l1, l2);
      if (dis < distance) {
        distance = dis;
      }
    }
    return distance;
  }

  @Override
  public float from_point(Translation2d point) {
    float distance = Float.MAX_VALUE;

    for (int i = 0; i < vertices.length - 1; i++) {
      float dis = PG_math.point_from_lineseg_f(vertices[i], vertices[(i + 1)], point);
      if (dis < distance) {
        distance = dis;
      }
    }
    return distance;
  }
}
