// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Robot {
    public static final double A_LENGTH = Units.inchesToMeters(27); // placeholder
    public static final double A_WIDTH = Units.inchesToMeters(27); // placeholder
    public static final double A_CROSS = Math.hypot(A_WIDTH, A_LENGTH);

    public static final double BUMPER = Units.inchesToMeters(3); // placeholder

    public static final double B_LENGTH = A_LENGTH + 2 * BUMPER;
    public static final double B_WIDTH = A_WIDTH + 2 * BUMPER;
    public static final double B_CROSS = Math.hypot(B_LENGTH, B_WIDTH);

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    public static final PIDTuning tuningMode = PIDTuning.NONE;

    public static final String rio = "rio";
    public static final String canivore = "canivore";
    public static final double loopPeriodSecs = 0.02;

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

  public static class Id {
    // all of these are placeholders until robot is configured
    public static final int kWrist = 15;
    public static final int kIntake = 18;
    public static final int kClimber = 16;
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

  public static class Controller {
    public static final int kPilotPort = 0;
    public static final int kCopilotPort = 1;

    public static final double DEADBAND = 0.1;
  }

  public static class IntakeConstants {
    // all of these are placeholders

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(kCurrentLimitsConfigs)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }

  public static class WristConstants {
    // again, more placeholders
    public static final double kTolerance = 0.2;
    public static final double kGearRatio = 25.0; // will change
    public static final int kCurrentLimit = 60;
  }

  public static class ClimberConstants {
    // all placeholders
    public static final double kTolerance = 0.2;
  }
}
