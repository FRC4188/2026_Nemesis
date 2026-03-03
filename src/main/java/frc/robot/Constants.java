// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

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

    public static final double PATH_ERROR = B_CROSS * 2.5;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final PIDTuning tuningMode = PIDTuning.DRIVE_MOD;
    public static RobotMode robotMode = RobotMode.NONE;

    public static final String rio = "rio";
    public static final String canivore = "canivore";
    public static final double loopPeriodSecs = 0.02;

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 34.261; // placeholder
    private static final double ROBOT_MOI = ROBOT_MASS_KG * B_CROSS * B_CROSS; // placeholer
    private static final double WHEEL_COF = 1.2; // how do you even calculate this

    public static final Translation3d RIGHT_CAMERA_FOCAL_TO_BOTTOM_SCREW =
        new Translation3d(
            Units.inchesToMeters((1.4025 - 0.0745) - 0.3625 / 2),
            Units.inchesToMeters(2.0740 / 2),
            Units.inchesToMeters(2.0445 / 2 - 0.6880 + 0.3625 / 2));

    public static final Translation3d LEFT_CAMERA_FOCAL_TO_BOTTOM_SCREW =
        new Translation3d(
            -Units.inchesToMeters((1.4025 - 0.0745) - 0.3625 / 2),
            -Units.inchesToMeters(2.0740 / 2),
            Units.inchesToMeters(2.0445 / 2 - 0.6880 + 0.3625 / 2));
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
    public static final double DRIVE_FF = 0.0;
    public static final double ANGLE_TOL = 0.2;
    public static final double DRIVE_TOL = 0.01;

    public static final double ANGLE_MAXVEL = 3.0 * Math.PI;
    public static final double ANGLE_MAXACC = 40.0;
    public static final ProfiledPIDController ANGLE_PID =
        (new ProfiledPIDController(
            5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(ANGLE_MAXVEL, ANGLE_MAXACC)));

    public static void updateAnglePID(double kP, double kI, double kD, double kF) {
      ANGLE_PID.setPID(kP, kI, kD);
    }

    public static final PIDController CORRECTION_PID = new PIDController(0.1, 0.0, 0.006);

    public static final RobotConfig PP_CONFIG =
        new RobotConfig(
            Robot.ROBOT_MASS_KG,
            Robot.ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                Robot.WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            frc.robot.subsystems.drive.Drive.getModuleTranslations());

    public static final double BLINE_DRIVE_TOL = 0.03;
    public static final Angle BLINE_ROT_TOL = Degrees.of(0.0349);
    public static final double BLINE_HANDOFF_RADIUS =
        0.1; // Increase for smooth motion / more inaccurate

    public static final double TRENCH_SAFETY_RADIUS = 1.6; // meters

    public static enum SafetyMode {
      NONE,
      LEFT_TRENCH,
      RIGHT_TRENCH
    }

    public static SafetyMode currentSafetyMode = SafetyMode.NONE;
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

  public static class Camera {
    public static final Translation3d cameraLeft = new Translation3d(Units.inchesToMeters(-11.29914), Units.inchesToMeters(-11.1000), Units.inchesToMeters(13.64718));
    public static final Translation3d cameraRight = new Translation3d(Units.inchesToMeters(-11.29914), Units.inchesToMeters(11.1000), Units.inchesToMeters(13.64718));
    public static final Translation3d cameraFront = new Translation3d(Units.inchesToMeters(7.89473), Units.inchesToMeters(9.73216), Units.inchesToMeters(7.44761));
  }
}
