// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static enum RobotMode {
    NONE,
    INTAKE,
    SHOOT,
    CLIMB
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum PIDTuning {
    NONE,
    DRIVE_MOD,
    TURN_MOD,
    ANGLE,
  }

  public static class Robot {
    public static final double A_LENGTH = Units.inchesToMeters(27);
    public static final double A_WIDTH = Units.inchesToMeters(27);
    public static final double A_CROSS = Math.hypot(A_WIDTH, A_LENGTH);

    public static final double BUMPER = Units.inchesToMeters(3); // placeholder

    public static final double B_LENGTH = A_LENGTH + 2 * BUMPER;
    public static final double B_WIDTH = A_WIDTH + 2 * BUMPER;
    public static final double B_CROSS = Math.hypot(B_LENGTH, B_WIDTH);

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final PIDTuning tuningMode = PIDTuning.NONE;
    public static final RobotMode robotMode = RobotMode.NONE;

    public static final String rio = "rio";
    public static final String canivore = "canivore";
    public static final double loopPeriodSecs = 0.02;
  }

  public static class Controller {
    public static final int kPilotPort = 0;
    public static final int kCopilotPort = 1;

    public static final double DEADBAND = 0.1;
  }

  public static class Drive {
    public static final double DRIVE_MAXVEL = 4.8;
    public static final double DRIVE_MAXACC = 8.0;
    public static final ProfiledPIDController DRIVE_PID =
        new ProfiledPIDController(
            5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(DRIVE_MAXVEL, DRIVE_MAXACC));

    public static final double ANGLE_FF = 2.0;
    public static final double ANGLE_TOL = 0.05;

    public static final double ANGLE_MAXVEL = 3.0 * Math.PI;
    public static final double ANGLE_MAXACC = 40.0;
    public static final ProfiledPIDController ANGLE_PID =
        (new ProfiledPIDController(
            5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(ANGLE_MAXVEL, ANGLE_MAXACC)));

    public static void updateAnglePID(double kP, double kI, double kD, double kF) {
      ANGLE_PID.setPID(kP, kI, kD);
    }
  }

  public static class Intake {
    public static final Translation2d location = new Translation2d(0, 0); // location
  }

  public static class Shooter {
    public static final Translation2d location =
        new Translation2d(-Robot.A_LENGTH / 2, 0); // placeholder
  }

  public static class Climber {
    public static final Translation2d location = new Translation2d(0, 0); // placeholder
  }
}
