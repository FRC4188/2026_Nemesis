package frc.robot.CSPLib.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;

public class RectFO extends PolygonFO {

  RectFO() {}

  public RectFO(float x, float y, float l, float w) {
    super(
        true,
        new Translation2d(x + 0.5 * l, y + 0.5 * w),
        new Translation2d(x + 0.5 * l, y + 0.5 * w),
        new Translation2d(x - 0.5 * l, y - 0.5 * w),
        new Translation2d(x + 0.5 * l, y - 0.5 * w));
  }
}
