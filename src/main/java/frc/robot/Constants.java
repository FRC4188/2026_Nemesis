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
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

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

    public static final double BUMPER = Units.inchesToMeters(3.5);

    public static final double B_LENGTH = A_LENGTH + 2 * BUMPER;
    public static final double B_WIDTH = A_WIDTH + 2 * BUMPER;
    public static final double B_CROSS = Math.hypot(B_LENGTH, B_WIDTH);

    public static final double PATH_ERROR = B_CROSS * 2.5;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static final CANBus rio = CANBus.roboRIO();
    public static final CANBus canivore = new CANBus("canivore");
    public static final double loopPeriodSecs = 0.02;

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = Units.lbsToKilograms(130);
    private static final double ROBOT_MOI = 6.5062;
    private static final double WHEEL_COF = 1.2; // how do you even calculate this
  }

  public static class Controller {
    public static final int kPilotPort = 0;
    public static final int kCopilotPort = 1;

    public static final double DEADBAND = 0.1;
  }

  public static class DriveConstants {
    public static final double DRIVE_MAXVEL = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double DRIVE_MAXACC = 8.0;
    public static final ProfiledPIDController DRIVE_PID =
        new ProfiledPIDController(
            5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(DRIVE_MAXVEL, DRIVE_MAXACC));

    public static final double ANGLE_FF = 2.0;
    public static final Rotation2d ANGLE_TOL = Rotation2d.fromDegrees(3.0);

    public static final double ANGLE_MAXVEL = DRIVE_MAXVEL / Drive.DRIVE_BASE_RADIUS;
    public static final double ANGLE_MAXACC = 40.0;
    public static final ProfiledPIDController ANGLE_PID =
        (new ProfiledPIDController(
            5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(ANGLE_MAXVEL, ANGLE_MAXACC)));

    public static void updateAnglePID(double kP, double kI, double kD, double kF) {
      ANGLE_PID.setPID(kP, kI, kD);
    }

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
  }

  public static class IntakeConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue = InvertedValue.Clockwise_Positive;
  }

  public static class WristConstants {
    public static final Rotation2d kTolerance = Rotation2d.fromDegrees(10.0);
    public static final double kGearRatio = 25.0;
    public static final Rotation2d Max_A = Rotation2d.fromDegrees(144.0);
    public static final Rotation2d Min_A = Rotation2d.fromDegrees(0.0);

    public static final double kStatorCurrent = 100; // placeholder
    public static final double kSupplyCurrent = 80; // placeholder
    public static final double kStallCurrent = 20; // placeholder
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
  }

  public static class ClimberConstants {
    public static final double kTolerance = Units.inchesToMeters(0.2);
    public static final double Max_H = Units.inchesToMeters(7.5);
    public static final double Min_H = 0.0;
    public static final double kGearBox = 27.0;
    public static final double kSproketDiameter = Units.inchesToMeters(0.75);
    public static final double kConversion = kGearBox / (kSproketDiameter * Math.PI);

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // paceholder
    public static final double kStallCurrent = 50.0; // paceholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final Slot0Configs climberGains =
        new Slot0Configs()
            .withKP(0.0) // 1.0
            .withKI(0.0)
            .withKD(0.0) // 0.3
            .withKG(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);
  }

  public static class HoodConstants {
    public static final double kGearRatio = 40.0;
    public static final Rotation2d kTolerance = Rotation2d.fromDegrees(1.0);
    public static final Rotation2d Max_A = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d Min_A = Rotation2d.fromDegrees(88.0);

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kStallCurrent = 20.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
    public static final Slot0Configs hoodGains =
        new Slot0Configs()
            .withKP(50.0)
            .withKI(5.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKG(0.4)
            .withGravityType(GravityTypeValue.Arm_Cosine);
  }

  public static class ShooterConstants {
    public static final double kTolerance = 100.0;
    public static final double kMinRPM = 1500.0;
    public static final double kMaxRPM = 4800.0;
    public static final double kGearRatio = 1.0;

    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder

    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kLeftInvertedValue = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kRightInvertedValue = InvertedValue.CounterClockwise_Positive;

    public static final Slot0Configs shooterGains =
        new Slot0Configs().withKP(10.0).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.5);
  }

  public static class IndexerConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Brake;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
  }

  public static class HopperConstants {
    public static final double kStatorCurrent = 100.0; // placeholder
    public static final double kSupplyCurrent = 80.0; // placeholder
    public static final double kPeakForwardTC = 50.0; // placeholder
    public static final double kPeakReverseTC = 50.0; // placeholder
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
    public static final InvertedValue kInvertedValue = InvertedValue.CounterClockwise_Positive;
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
}
