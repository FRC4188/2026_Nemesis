package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class SimulationConfig {

  public static final Joint wrist =
      new Joint(
          // all values but mass are arbitrary and subject to change
          Kilograms.of(Units.lbsToKilograms(13.0005)).magnitude(),
          Meters.of(0).magnitude(),
          KilogramSquareMeters.of(0).magnitude(),
          Meters.of(0).magnitude());

  public static final Joint hood =
      new Joint(
          // all values but mass are arbitrary and subject to change
          Kilograms.of(Units.lbsToKilograms(9.6406)).magnitude(),
          Meters.of(0).magnitude(),
          KilogramSquareMeters.of(0).magnitude(),
          Meters.of(0).magnitude());

  // config.json has all of these at zero, these are values from the robots's origin - Alex R.
  public static Pose3d origin = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  public static Pose3d wristAxis =
      new Pose3d(13.625000, -9.155369 + 2.37851, 1.690000, new Rotation3d(0, 0, 0));
  public static Pose3d hoodAxis =
      new Pose3d(-9.625000, 9.00780 - 2.855782, 14.64837 - 0.142798, new Rotation3d(0, 0, 0));
  public static Pose3d climberAxis =
      new Pose3d(12.875000 - 0.937500, 0.687500 - 0.937500, 3.690000, new Rotation3d());
  public static Pose3d hopper = new Pose3d(0, -3.850000 + 3.975000, 1.690000, new Rotation3d());
  public static Pose3d agitator = new Pose3d(6.940000, 6.491600, 3.815000, new Rotation3d());
  public static Pose3d indexer = new Pose3d(0, 7.625000 - 0.125000, 3.877500, new Rotation3d());

  public record Joint(double mass, double length, double inertiaAbtCoM, double disFromPivot2CoG) {}
}
