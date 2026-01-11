package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants { // first time going off of an onshape cad of field
    public final static double field_length = Units.inchesToMeters(651.2);
    public final static double field_width = Units.inchesToMeters(317.7);

    public static class Trench {

        public static final double trench_width = Units.inchesToMeters(60.16 - 9.81);
        public static final double trench_length = Units.inchesToMeters(6);

        public static final Translation2d right_trench_center = new Translation2d(Units.inchesToMeters(180.1) + trench_length / 2, trench_width / 2);
        public static final Translation2d left_trench_center = new Translation2d(Units.inchesToMeters(180.1) + trench_length / 2, Units.inchesToMeters(267.34) + trench_width / 2);

        // Optimal Entrances
        public static final Translation2d right_trench_alliance_entrance = new Translation2d(Units.inchesToMeters(156.61), trench_width / 2);
        public static final Translation2d left_trench_alliance_entrance = new Translation2d(Units.inchesToMeters(156.61), Units.inchesToMeters(267.34) + trench_width / 2);

        public static final Translation2d right_trench_neutral_entrance = new Translation2d(Units.inchesToMeters(204.18), trench_width / 2);
        public static final Translation2d left_trench_neutral_entrance = new Translation2d(Units.inchesToMeters(204.18), Units.inchesToMeters(267.34) + trench_width / 2);
    }

    public static class Bump {
        public static final double bump_wall_width = Units.inchesToMeters(9.81);

        public static final double bump_width = Units.inchesToMeters(85.11);
        public static final double bump_wo_wall_width = Units.inchesToMeters(73.5);
        public static final double bump_length = Units.inchesToMeters(44.39);

        // Right Bump (off DriveStation)
        public static final Translation2d right_bump_right_close_corner = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(50.53));
        public static final Translation2d right_bump_left_close_corner = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(50.53) + bump_width);
        public static final Translation2d right_bump_right_far_corner = new Translation2d(Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(50.53));
        public static final Translation2d right_bump_left_far_corner = new Translation2d(Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(50.53) + bump_width);

        // Left Bump (off DriveStation)
        public static final Translation2d left_bump_right_close_corner = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(182.34));
        public static final Translation2d left_bump_left_close_corner = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(182.34) + bump_width);
        public static final Translation2d left_bump_right_far_corner = new Translation2d(Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(182.34));
        public static final Translation2d left_bump_left_far_corner = new Translation2d(Units.inchesToMeters(158.61) + bump_length, Units.inchesToMeters(182.34) + bump_width);

        // Optimal Entrances
        public static final Translation2d right_bump_alliance_entrance = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters((62.37 + 135.34) / 2));
        public static final Translation2d right_bump_neutral_entrance = new Translation2d(Units.inchesToMeters(204.18), Units.inchesToMeters((62.37 + 135.34) / 2));
        public static final Translation2d left_bump_alliance_entrance = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters((182.59 + 255.31) / 2));
        public static final Translation2d left_bump_neutral_entrance = new Translation2d(Units.inchesToMeters(204.18), Units.inchesToMeters((182.59 + 255.31) / 2));

    }

    public static class Hub {
        public static final double hub_width = Units.inchesToMeters(47);
        public static final double hub_length = Units.inchesToMeters(47);

        public static final Translation2d right_close_corner = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(135.34));
        public static final Translation2d left_close_corner = new Translation2d(Units.inchesToMeters(158.61), Units.inchesToMeters(135.34) + hub_width);
        public static final Translation2d right_far_corner = new Translation2d(Units.inchesToMeters(158.61) + hub_length, Units.inchesToMeters(135.34));
        public static final Translation2d left_far_corner = new Translation2d(Units.inchesToMeters(158.61) + hub_length, Units.inchesToMeters(135.34) + hub_length);

        // Optimal Scoring Location
        public static final Translation2d hub_center = new Translation2d(Units.inchesToMeters(158.61) + hub_length / 2, Units.inchesToMeters(135.34) + hub_width / 2);
        public static final double opening_radius = Units.inchesToMeters(27.62 / 2);
    }

    public static class FuelField {
        public static final double fuelfield_width = Units.inchesToMeters(176);
        public static final double fuelfield_length = Units.inchesToMeters(66);

        public static final Translation2d right_close_corner = new Translation2d(Units.inchesToMeters(289.66), Units.inchesToMeters(67.89));
        public static final Translation2d left_close_corner = new Translation2d(Units.inchesToMeters(289.66), Units.inchesToMeters(67.89) + fuelfield_width);
        public static final Translation2d right_far_corner = new Translation2d(Units.inchesToMeters(289.66) + fuelfield_length, Units.inchesToMeters(67.89));
        public static final Translation2d left_far_corner = new Translation2d(Units.inchesToMeters(289.66) + fuelfield_length, Units.inchesToMeters(67.89) + fuelfield_width);
    }

    public static class Tower {
        public final static Translation2d right_close_corner = new Translation2d(0, Units.inchesToMeters(128.04));
        public final static Translation2d left_close_corner = new Translation2d(0, Units.inchesToMeters(166.89));
        public final static Translation2d right_far_corner = new Translation2d(Units.inchesToMeters(45.1), Units.inchesToMeters(128.04));
        public final static Translation2d left_far_corner = new Translation2d(Units.inchesToMeters(45.1), Units.inchesToMeters(166.89));


    }

    public static class Depot {
        public final static Translation2d right_close_corner = new Translation2d(0, Units.inchesToMeters(213.84));
        public final static Translation2d left_close_corner = new Translation2d(0, Units.inchesToMeters(255.84));
        public final static Translation2d right_far_corner = new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(213.84));
        public final static Translation2d left_far_corner = new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(255.84));
    }


}
