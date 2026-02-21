// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

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
    public static final int kWrist = 13;
    public static final int kIntake = 14;
    // public static final int kIntakeCANCoder = 15;
    public static final int kHopper = 16;
    public static final int kIndexer = 17;
    public static final int kRightShooter = 18;
    public static final int kLeftShooter = 19;
    public static final int kHood = 20;
    // public static final int kHoodCANCoder = 21;
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

  public static class Controller {
    public static final int kPilotPort = 0;
    public static final int kCopilotPort = 1;

    public static final double DEADBAND = 0.1;
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
    public static final NeutralModeValue kNuetralMode = NeutralModeValue.Coast;
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
    public static final double kConverter = 1.0; // placeholder
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
    public static final double Max_A = Units.degreesToRadians(82);
    public static final double Min_A = Units.degreesToRadians(0.0);

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
    public static final double kTolerance = 100.0; // rpm units
    public static final double kLowVel = 500.0;
    public static final double kMiddleVel = 1000.0;
    public static final double kHighVel = 2000.0;
    public static final double kGearRatio = 1.0;

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

    public static final Slot0Configs shooterGains =
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
