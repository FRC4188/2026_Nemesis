// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static enum RobotMode {
    NONE,
    INTAKE,
    SHOOT,
    CLIMB
  }

  public static class Id {
    public static final int kWrist = 13;
    public static final int kIntake = 14;
    // public static final int kIntakeCANCoder = 15;
    public static final int kHopper = 16;
    public static final int kIndexer = 17;
    public static final int kRightShooter = 18;
    public static final int kLeftShooter = 19;
    public static final int kHood = 20;
    public static final int kHoodCANCoder = 21;
    public static final int kClimber = 22;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum PIDTuning {
    NONE,
    DRIVE_MOD,
    TURN_MOD,
    ANGLE,
    SHOOTER,
    HOOD,
    WRIST
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

    public static final CANBus rio = CANBus.roboRIO();
    public static final CANBus canivore = new CANBus("canivore");
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
    public static final double ANGLE_TOL = 0.2;

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
  }

  public static class IntakeConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class WristConstants {
    public static final double kTolerance = 0.2;
    public static final double kGearRatio = 25.0;
    public static final double Max_A = Units.degreesToRadians(144);
    public static final double Min_A = Units.degreesToRadians(0.0);

    public static final double kStatorCurrent = 100; // placeholder
    public static final double kSupplyCurrent = 80; // placeholder
    public static final double kStallCurrent = 50; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode =
        NeutralModeValue.Coast; // free pivot is faster
    public static final InvertedValue kInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final Slot0Configs wristGains =
        new Slot0Configs()
            .withKP(0.0) // placeholder
            .withKI(0.0) // placeholder
            .withKD(0.0) // placeholder
            .withKS(0.0) // testing usefulness
            .withKG(0.0) // placeholder
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // public static final double kCanCoderOffset = 0.0;
    // public static final SensorDirectionValue kDirection =
    // SensorDirectionValue.Clockwise_Positive;
  }

  public static class ClimberConstants {
    public static final double kTolerance = 0.2;
    public static final double kGearRatio = 45.0;
    public static final double kConverter = 0.75 * Math.PI; // placeholder
    public static final double Max_H = 1.0; // placeholder
    public static final double Min_H = 0.0;

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // paceholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final Slot0Configs climberGains =
        new Slot0Configs()
            .withKP(0.0) // placeholder
            .withKI(0.0) // placeholder
            .withKD(0.0) // placeholder
            .withKS(0.0) // testing usefulness
            .withKG(0.0) // placeholder
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final ClosedLoopOutputType motorClosedLoopOutput =
        ClosedLoopOutputType.TorqueCurrentFOC;
  }

  public static class HoodConstants {
    public static final double kGearBox = 16.0;
    public static final double kSproket = 3.0;
    public static final double kGearRatio = kGearBox * kSproket;
    public static final double kTolerance = 0.01;
    public static final double Max_A = Units.degreesToRadians(90.0);
    public static final double Min_A = Units.degreesToRadians(8.0);

    public static final double kStatorCurrent = 100; // placeholder
    public static final double kSupplyCurrent = 80; // placeholder
    public static final double kStallCurrent = 50; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final Slot0Configs hoodGains =
        new Slot0Configs()
            .withKP(0.0) // placeholder
            .withKI(0.0) // placeholder
            .withKD(0.0) // placeholder
            .withKS(0.0) // testing usefulness
            .withKG(0.0) // placeholder
            .withGravityType(GravityTypeValue.Arm_Cosine); // TODO: workaround 1:1

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;

    public static final double kCanCoderOffset = 0.0; // placeholder
    public static final SensorDirectionValue kDirection =
        SensorDirectionValue.Clockwise_Positive; // placeholder
  }

  public static class ShooterConstants {
    public static final Translation3d location =
        new Translation3d(-Robot.A_LENGTH / 2, 0, 20.0); // placeholder
    public static final double kWheelDiam = Units.inchesToMeters(4.0);

    public static final double kTolerance = 100.0; // rpm units
    public static final double kLowVel = 500.0; // placeholder
    public static final double kMiddleVel = 1000.0; // placeholder
    public static final double kHighVel = 2000.0; // placeholder
    public static final double kGearRatio = 1.0;
    public static final double kDropVel = 1.0; // placeholder

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder

    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kLeftInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final InvertedValue kRightInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder

    public static final Slot0Configs rightShooterGains =
        new Slot0Configs()
            .withKP(0.0) // placeholder
            .withKI(0.0) // placeholder
            .withKD(0.0) // placeholder
            .withKS(0.0) // testing usefulness
            .withKV(0.0); // placeholder

    public static final Slot0Configs leftShooterGains =
        new Slot0Configs()
            .withKP(0.0) // placeholder
            .withKI(0.0) // placeholder
            .withKD(0.0) // placeholder
            .withKS(0.0) // testing usefulness
            .withKV(0.0); // placeholder

    public static final ClosedLoopOutputType motorClosedLoopOutput =
        ClosedLoopOutputType.TorqueCurrentFOC;
  }

  public static class IndexerConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class HopperConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue =
        InvertedValue.Clockwise_Positive; // placeholder
    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }
}
