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
      new Pose3d(
          Units.inchesToMeters(11.375000), 0, Units.inchesToMeters(9.337049), new Rotation3d());

  public static Pose3d hoodAxis =
      new Pose3d(
          Units.inchesToMeters(-4.145811),
          0,
          Units.inchesToMeters(3.128070 + 11.37750),
          new Rotation3d());

  //   public static Pose3d climberAxis =
  //       new Pose3d(
  //           Units.inchesToMeters(12.875000 - 0.937500),
  //           Units.inchesToMeters(0.687500 - 0.937500),
  //           Units.inchesToMeters(3.690000),
  //           new Rotation3d());
  //   public static Pose3d climberAxis = new Pose3d(Units.inchesToMeters(-0.750000),
  // Units.inchesToMeters(-11.000000), Units.inchesToMeters(3.690000), new Rotation3d());
  //   public static Pose3d climberAxis = new Pose3d(0, 0, 0, new Rotation3d());
  public static Pose3d climberAxis =
      new Pose3d(
          Units.inchesToMeters(0.250000),
          Units.inchesToMeters(11.9375),
          Units.inchesToMeters(3.690000),
          new Rotation3d());

  //   public static Pose3d hopper =
  //       new Pose3d(
  //           Units.inchesToMeters(0),
  //           Units.inchesToMeters(4.100000 - 3.975000),
  //           Units.inchesToMeters(1.690000),
  //           new Rotation3d(0, -Math.PI / 2, 0));
  public static Pose3d hopper =
      new Pose3d(
          Units.inchesToMeters(3.850000 - 3.975000),
          0,
          Units.inchesToMeters(1.690000),
          new Rotation3d());
  //   public static Pose3d hopper = new Pose3d(0, 0, 0, new Rotation3d());

  //   public static Pose3d agitator =
  //       new Pose3d(
  //           Units.inchesToMeters(6.940000),
  //           Units.inchesToMeters(6.491600),
  //           Units.inchesToMeters(3.815000),
  //           new Rotation3d());
  public static Pose3d agitator =
      new Pose3d(
          Units.inchesToMeters(-6.491600),
          Units.inchesToMeters(6.940000),
          Units.inchesToMeters(3.815000),
          new Rotation3d());
  public static Pose3d indexer =
      new Pose3d(
          Units.inchesToMeters(-7.500000), 0, Units.inchesToMeters(3.877500), new Rotation3d());
  //   public static Pose3d indexer = new Pose3d(0, 0, 0, new Rotation3d());

  public record Joint(double mass, double length, double inertiaAbtCoM, double disFromPivot2CoG) {}
}
