// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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

  public static class Id {
    public static final int kWrist = 13;
    public static final int kIntake = 14;
    public static final int kHopper = 16;
    public static final int kIndexer = 17;
    public static final int kRightShooter = 18;
    public static final int kLeftShooter = 19;
    public static final int kHood = 20;
    public static final int kClimber = 22;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
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

    public static final CANBus rio = CANBus.roboRIO();
    public static final CANBus canivore = new CANBus("canivore");
    public static final double loopPeriodSecs = 0.02;

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 130; // placeholder
    private static final double ROBOT_MOI = ROBOT_MASS_KG * B_CROSS * B_CROSS; // placeholer
    private static final double WHEEL_COF = 1.2; // how do you even calculate this
  }

  public static class Controller {
    public static final int kPilotPort = 0;
    public static final int kCopilotPort = 1;

    public static final double DEADBAND = 0.1;
  }

  public static class DriveConstants {
    public static final double DRIVE_MAXVEL = 4.5;
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

  public static class IntakeConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
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
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
    public static final Slot0Configs wristGains =
        new Slot0Configs()
            .withKP(10.0)
            .withKI(6.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKG(0.4)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class ClimberConstants {
    public static final double kTolerance = 5.0;
    public static final double Max_R = 85.0;
    public static final double Min_R = 0.0;

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // paceholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final Slot0Configs climberGains =
        new Slot0Configs()
            .withKP(1.0)
            .withKI(0.0)
            .withKD(0.3)
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final ClosedLoopOutputType motorClosedLoopOutput =
        ClosedLoopOutputType.TorqueCurrentFOC;
  }

  public static class HoodConstants {
    public static final double kGearRatio = 40.0;
    public static final double kTolerance = 0.05;
    public static final double Max_A = Units.degreesToRadians(90.0);
    public static final double Min_A = Units.degreesToRadians(8.0);

    public static final double kStatorCurrent = 100; // placeholder
    public static final double kSupplyCurrent = 80; // placeholder
    public static final double kStallCurrent = 50; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final Slot0Configs hoodGains =
        new Slot0Configs()
            .withKP(25.0)
            .withKI(5.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKG(0.4)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class ShooterConstants {
    public static final Translation3d location =
        new Translation3d(-Robot.A_LENGTH / 2, 0, 22.0); // placeholder
    public static final double kWheelDiam = Units.inchesToMeters(4.0);

    public static final double kTolerance = 240.0;
    public static final double kLowVel = 1800.0;
    public static final double kMiddleVel = 2400.0;
    public static final double kHighVel = 4800.0;
    public static final double kGearRatio = 1.0;
    public static final double kDropVel = 600.0; // placeholder

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder

    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kLeftInvertedValue = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRightInvertedValue = InvertedValue.CounterClockwise_Positive;

    public static final Slot0Configs rightShooterGains =
        new Slot0Configs().withKP(10.0).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.5);

    public static final Slot0Configs leftShooterGains =
        new Slot0Configs().withKP(10.0).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.5);

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
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class HopperConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 50.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class CameraConstants {
    // public static final Transform3d cameraLeft =
    //     new Transform3d(
    //         Units.inchesToMeters(-11.29914),
    //         Units.inchesToMeters(11.1000),
    //         Units.inchesToMeters(13.64718),
    //         new Rotation3d(0, 0, Math.PI / 2));

    public static final Transform3d cameraLeft =
        new Transform3d(
            Units.inchesToMeters(-11.500000),
            Units.inchesToMeters(11.100000 - 1.600000),
            Units.inchesToMeters(13.889783),
            new Rotation3d(0, 0, Math.PI / 2));

    // public static final Transform3d cameraRight =
    //     new Transform3d(
    //         Units.inchesToMeters(-11.29914),
    //         Units.inchesToMeters(-11.1000),
    //         Units.inchesToMeters(13.64718),
    //         new Rotation3d(0, 0, -Math.PI / 2));

    public static final Transform3d cameraRight =
        new Transform3d(
            Units.inchesToMeters(-11.500000),
            Units.inchesToMeters(-11.100000 + 1.600000),
            Units.inchesToMeters(13.889783),
            new Rotation3d(0, 0, -Math.PI / 2));

    // public static final Transform3d cameraFront =
    //     new Transform3d(
    //         Units.inchesToMeters(7.89473),
    //         Units.inchesToMeters(9.73216),
    //         Units.inchesToMeters(7.44761),
    //         new Rotation3d(0, 0, 0));

    public static final Transform3d cameraFront =
        new Transform3d(
            Units.inchesToMeters(8.812214),
            Units.inchesToMeters(-9.982283),
            Units.inchesToMeters(8.205321),
            new Rotation3d(0, 0, 0));
  }

  public static class Camera {
    public static final Translation3d cameraLeft =
        new Translation3d(
            Units.inchesToMeters(-11.29914),
            Units.inchesToMeters(-11.1000),
            Units.inchesToMeters(13.64718));
    public static final Translation3d cameraRight =
        new Translation3d(
            Units.inchesToMeters(-11.29914),
            Units.inchesToMeters(11.1000),
            Units.inchesToMeters(13.64718));
    public static final Translation3d cameraFront =
        new Translation3d(
            Units.inchesToMeters(7.89473),
            Units.inchesToMeters(9.73216),
            Units.inchesToMeters(7.44761));
  }
}
