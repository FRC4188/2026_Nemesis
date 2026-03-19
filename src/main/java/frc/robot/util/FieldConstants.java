package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.List;

public final class FieldConstants { // going off of an onshape cad of field
  public static final double field_length = Units.inchesToMeters(651.22);
  public static final double field_width = Units.inchesToMeters(317.69);
  public static final Translation2d field_center =
      new Translation2d(field_length / 2, field_width / 2);

  public static final double alliance_zone_x = Units.inchesToMeters(156.61);

  public static final Translation2d right_alliance_shoot =
      new Translation2d(Units.inchesToMeters(156.61) / 2, field_width / 4);
  public static final Translation2d left_alliance_shoot =
      new Translation2d(Units.inchesToMeters(156.61) / 2, 3 * field_width / 4);
  public static final Translation2d center_alliance_shoot =
      new Translation2d(Units.inchesToMeters(156.61) / 2, field_width / 2);
  //   public static final Translation2d alliance_area_center =
  //     new Translation2d()

  public static class Trench {

    public static final double trench_width = Units.inchesToMeters(60.16 - 9.81);
    public static final double trench_length = Units.inchesToMeters(6);

    public static final Translation2d right_trench_center =
        new Translation2d(Units.inchesToMeters(180.1) + trench_length / 2, trench_width / 2);
    public static final Translation2d left_trench_center =
        new Translation2d(
            Units.inchesToMeters(180.1) + trench_length / 2,
            Units.inchesToMeters(267.34) + trench_width / 2);

    // Optimal Entrances
    public static final Translation2d right_trench_alliance_entrance =
        new Translation2d(Units.inchesToMeters(156.61), trench_width / 2);
    public static final Translation2d left_trench_alliance_entrance =
        new Translation2d(
            Units.inchesToMeters(156.61), Units.inchesToMeters(267.34) + trench_width / 2);

    public static final Translation2d right_trench_neutral_entrance =
        new Translation2d(Units.inchesToMeters(204.18), trench_width / 2);
    public static final Translation2d left_trench_neutral_entrance =
        new Translation2d(
            Units.inchesToMeters(204.18), Units.inchesToMeters(267.34) + trench_width / 2);

    public static final Translation2d right_trench_alliance_preentrance =
        new Translation2d(Units.inchesToMeters(156.61) - 1.5, trench_width / 2);
    public static final Translation2d left_trench_alliance_preentrance =
        new Translation2d(
            Units.inchesToMeters(156.61) - 1.5, Units.inchesToMeters(267.34) + trench_width / 2);

    public static final Translation2d right_trench_neutral_preentrance =
        new Translation2d(Units.inchesToMeters(204.18) + 1.5, trench_width / 2);
    public static final Translation2d left_trench_neutral_preentrance =
        new Translation2d(
            Units.inchesToMeters(204.18) + 1.5, Units.inchesToMeters(267.34) + trench_width / 2);

    // just further out than preentrance
    public static final Translation2d left_trench_alliance_approach =
        new Translation2d(
            Units.inchesToMeters(156.61) - Constants.Robot.B_CROSS,
            Units.inchesToMeters(267.34) + trench_width / 2);
    public static final Translation2d right_trench_alliance_approach =
        new Translation2d(Units.inchesToMeters(156.61) - Constants.Robot.B_CROSS, trench_width / 2);
    public static final Translation2d left_trench_neutral_approach =
        new Translation2d(
            Units.inchesToMeters(204.18) + Constants.Robot.B_CROSS,
            Units.inchesToMeters(267.34 + trench_width / 2));
    public static final Translation2d right_trench_neutral_approach =
        new Translation2d(Units.inchesToMeters(204.18) + Constants.Robot.B_CROSS, trench_width / 2);
  }

  public static class Bump {
    public static final double bump_wall_width = Units.inchesToMeters(9.81);

    public static final double bump_width = Units.inchesToMeters(85.11);
    public static final double bump_wo_wall_width = Units.inchesToMeters(73.5);
    public static final double bump_length = Units.inchesToMeters(44.39);

    // Right Bump (off DriveStation)
    public static final Translation2d right_bump_right_close_corner =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(50.53));
    public static final Translation2d right_bump_left_close_corner =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(50.53) + bump_width);
    public static final Translation2d right_bump_right_far_corner =
        new Translation2d(Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(50.53));
    public static final Translation2d right_bump_left_far_corner =
        new Translation2d(
            Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(50.53) + bump_width);

    // Left Bump (off DriveStation)
    public static final Translation2d left_bump_right_close_corner =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(182.34));
    public static final Translation2d left_bump_left_close_corner =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(182.34) + bump_width);
    public static final Translation2d left_bump_right_far_corner =
        new Translation2d(Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(182.34));
    public static final Translation2d left_bump_left_far_corner =
        new Translation2d(
            Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(182.34) + bump_width);

    // Optimal Entrances
    public static final Translation2d right_bump_alliance_entrance =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters((62.37 + 135.34) / 2));
    public static final Translation2d right_bump_neutral_entrance =
        new Translation2d(Units.inchesToMeters(204.18), Units.inchesToMeters((62.37 + 135.34) / 2));
    public static final Translation2d left_bump_alliance_entrance =
        new Translation2d(
            Units.inchesToMeters(158.61), Units.inchesToMeters((182.59 + 255.31) / 2));
    public static final Translation2d left_bump_neutral_entrance =
        new Translation2d(
            Units.inchesToMeters(204.18), Units.inchesToMeters((182.59 + 255.31) / 2));
  }

  public static class Hub {
    public static final double hub_width = Units.inchesToMeters(47);
    public static final double hub_length = Units.inchesToMeters(47);

    public static final Translation2d right_close_corner =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(135.34));
    public static final Translation2d left_close_corner =
        new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(135.34) + hub_width);
    public static final Translation2d right_far_corner =
        new Translation2d(Units.inchesToMeters(158.61) + hub_length, Units.inchesToMeters(135.34));
    public static final Translation2d left_far_corner =
        new Translation2d(
            Units.inchesToMeters(158.61) + hub_length, Units.inchesToMeters(135.34) + hub_length);

    // Optimal Scoring Location
    public static final Translation3d hub_center =
        new Translation3d(
            Units.inchesToMeters(158.61) + hub_length / 2,
            Units.inchesToMeters(135.34) + hub_width / 2,
            Units.inchesToMeters(71.87));

    public static final Translation2d hub_center_2d =
        new Translation2d(
            Units.inchesToMeters(158.61) + hub_length / 2,
            Units.inchesToMeters(135.34) + hub_width / 2);

    public static final double small_opening_radius = Units.inchesToMeters(27.62 / 2);
    public static final double large_opening_radius = Units.inchesToMeters(48.19 / 2);

    public static final Translation2d hub_aim =
        new Translation2d(
            Units.inchesToMeters(158.61) + hub_length / 2,
            Units.inchesToMeters(135.34) + hub_width / 2);
  }

  public static class FuelField {
    // uses the fuel ball centers NOT edges
    public static final double fuelfield_width = Units.inchesToMeters(176);
    public static final double fuelfield_length = Units.inchesToMeters(66);

    public static final Translation2d right_close_corner =
        new Translation2d(Units.inchesToMeters(289.66), Units.inchesToMeters(67.89));
    public static final Translation2d left_close_corner =
        new Translation2d(
            Units.inchesToMeters(289.66), Units.inchesToMeters(67.89) + fuelfield_width);
    public static final Translation2d right_far_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length, Units.inchesToMeters(67.89));
    public static final Translation2d left_far_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length,
            Units.inchesToMeters(67.89) + fuelfield_width);
    public static final Translation2d right_midline_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length / 2, Units.inchesToMeters(67.89));
    public static final Translation2d left_midline_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length / 2,
            Units.inchesToMeters(67.89) + fuelfield_width);
    public static final Translation2d left_close_corner_approach =
        new Translation2d(
            Units.inchesToMeters(289.66) + Constants.Robot.B_CROSS / 2.5,
            Units.inchesToMeters(67.89) + fuelfield_width + Constants.Robot.B_CROSS / 2.5);
    public static final Translation2d right_close_corner_approach =
        new Translation2d(
            Units.inchesToMeters(289.66) + Constants.Robot.B_CROSS / 2.5,
            Units.inchesToMeters(67.89) - Constants.Robot.B_CROSS / 2.5);

    public static final Translation2d middle_close_line =
        new Translation2d(
            Units.inchesToMeters(289.66), Units.inchesToMeters(67.89) + fuelfield_width / 2);
    public static final Translation2d middle_far_line =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length,
            Units.inchesToMeters(67.89) + fuelfield_width / 2);

    public static final Translation2d intake_midline =
        new Translation2d(
            FieldConstants.field_length / 2 - Constants.Robot.B_LENGTH / 2,
            Units.inchesToMeters(67.89) + fuelfield_width / 2);
    public static final Translation2d intake_closeline =
        new Translation2d(
            Units.inchesToMeters(289.66) - Constants.Robot.B_LENGTH / 2,
            Units.inchesToMeters(67.89) + fuelfield_width / 2);

    public static final Translation2d intake_right_close_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) - Constants.Robot.B_LENGTH / 2,
            Units.inchesToMeters(67.89));
    public static final Translation2d intake_left_close_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) - Constants.Robot.B_LENGTH / 2,
            Units.inchesToMeters(67.89) + fuelfield_width);

    public static final Translation2d intake_right_midline_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length / 2 - Constants.Robot.B_LENGTH / 2,
            Units.inchesToMeters(67.89));
    public static final Translation2d intake_left_midline_corner =
        new Translation2d(
            Units.inchesToMeters(289.66) + fuelfield_length / 2 - Constants.Robot.B_LENGTH / 2,
            Units.inchesToMeters(67.89) + fuelfield_width);
  }

  public static class Tower {
    public static final Translation2d right_close_corner =
        new Translation2d(0, Units.inchesToMeters(128.04));

    public static final Translation2d left_close_corner =
        new Translation2d(0, Units.inchesToMeters(166.89));
    public static final Translation2d right_far_corner =
        new Translation2d(Units.inchesToMeters(45.1), Units.inchesToMeters(128.04));
    public static final Translation2d left_far_corner =
        new Translation2d(Units.inchesToMeters(45.1), Units.inchesToMeters(166.89));

    public static final Translation2d left_approach_pos = new Translation2d(0.85, 4.85);
    public static final Translation2d left_intermediate_approach_pos =
        new Translation2d(0.85, 4.62);
    public static final Translation2d left_inter_inter_approach_pos = new Translation2d(1, 4.62);
    public static final Translation2d left_align_pos = new Translation2d(1, 4.562);

    public static final Translation2d right_approach_pos = new Translation2d(1.5, 1.750);
    public static final Translation2d right_intermediate_approach_pos =
        new Translation2d(1.3, 2.85);
    public static final Translation2d right_inter_inter_approach_pos =
        new Translation2d(1.112, 2.85);
    public static final Translation2d right_align_pos =
        new Translation2d(1.112 - Units.inchesToMeters(1.5), 2.924);

    public static final List<Waypoint> left_approach =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.589, 5, Rotation2d.k180deg), // placeholder
            new Pose2d(1, 4.9, Rotation2d.k180deg),
            new Pose2d(1, 4.562, Rotation2d.k180deg) // placeholder
            );

    public static final List<Waypoint> right_approach =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.589, 2.2, Rotation2d.k180deg), // placeholder
            new Pose2d(1.112, 2.5, Rotation2d.k180deg),
            new Pose2d(1.112, 2.924, Rotation2d.k180deg) // placeholder
            );
  }

  public static class Depot {
    public static final double inside_length = Units.inchesToMeters(24);
    public static final double outside_length = Units.inchesToMeters(27);

    public static final double inside_width = Units.inchesToMeters(36);
    public static final double outside_width = Units.inchesToMeters(42);

    public static final Translation2d right_close_corner =
        new Translation2d(0, Units.inchesToMeters(213.84));
    public static final Translation2d left_close_corner =
        new Translation2d(0, Units.inchesToMeters(255.84));
    public static final Translation2d right_far_corner =
        new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(213.84));
    public static final Translation2d left_far_corner =
        new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(255.84));
    public static final Translation2d close_middle_edge =
        new Translation2d(0, Units.inchesToMeters(234.84));
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(13.5), Units.inchesToMeters(234.84));
    public static final Translation2d far_middle_edge =
        new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(234.84));

    // weird name i know. positions the robot such that its diagonal is right at the depot corner
    public static final Translation2d left_far_corner_bot_diag_offset =
        new Translation2d(
            Units.inchesToMeters(27) + Constants.Robot.B_CROSS / 2,
            Units.inchesToMeters(255.84) + Constants.Robot.B_CROSS / 2);
    public static final Translation2d right_far_corner_bot_diag_offset =
        new Translation2d(
            Units.inchesToMeters(27) + Constants.Robot.B_CROSS / 2,
            Units.inchesToMeters(213.84) - Constants.Robot.B_CROSS / 2);
    public static final Translation2d left_far_corner_bot_side_offset =
        new Translation2d(
            Units.inchesToMeters(27) + Constants.Robot.B_CROSS / 2, Units.inchesToMeters(255.84));
    public static final Translation2d right_far_corner_bot_side_offset =
        new Translation2d(
            Units.inchesToMeters(27) + Constants.Robot.B_CROSS / 2, Units.inchesToMeters(213.84));
  }
}
