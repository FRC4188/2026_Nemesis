// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ID {
    //TODO: set pins later
    public static final double shooter_wheel_top = 0;
    public static final double shooter_wheel_bottom = 0;
    //add other stuff later
  }

  public static class Robot {
    public static final double A_LENGTH = Units.inchesToMeters(30); //placeholder
    public static final double A_WIDTH = Units.inchesToMeters(30); //placeholder
    public static final double A_CROSS = Math.hypot(A_WIDTH, A_LENGTH);

    public static final double BUMPER = Units.inchesToMeters(3); //placeholder

    public static final double B_LENGTH = A_LENGTH + 2*BUMPER;
    public static final double B_WIDTH = A_WIDTH + 2*BUMPER;
    public static final double B_CROSS = Math.hypot(B_LENGTH, B_WIDTH);

  }

  public static class Controller {
    public static final int kPilotPort = 0;
    public static final int kCopilotPort = 1;

    public static final double DEADBAND = 0.1;

  }

  public static class Shooter {
    public static final TalonFXConfiguration config = new TalonFXConfiguration();
  }
}
