// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble() * Constants.Drive.DRIVE_MAXVEL,
                  ySupplier.getAsDouble() * Constants.Drive.DRIVE_MAXVEL,
                  omegaSupplier.getAsDouble() * Constants.Drive.DRIVE_MAXACC);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds, AllianceFlip.apply(drive.getRotation())));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    ProfiledPIDController angleController = Constants.Drive.ANGLE_PID;
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              if (Constants.Robot.tuningMode == Constants.PIDTuning.ANGLE) {
                Logger.recordOutput("Drive/Target Angle", rotationSupplier.get().getRadians());
                Logger.recordOutput(
                  "Drive/Current Angle", drive.getPose().getRotation().getRadians());
              }

              double omega =
                  angleController.calculate(
                          drive.getRotation().getRadians(), rotationSupplier.get().getRadians())
                      + angleController.getSetpoint().velocity * Constants.Drive.ANGLE_FF;

              boolean withinTolerance = Math.abs(drive.getRotation().getRadians() - rotationSupplier.get().getRadians())
                      < Constants.Drive.ANGLE_TOL
                  && angleController.getSetpoint().velocity == 0.0;

              Logger.recordOutput("Drive/At Setpoint?",withinTolerance);
              if (withinTolerance) omega = 0.0;

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                      ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds, AllianceFlip.apply(drive.getRotation())));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveAtAngle(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    // placeholder
    Supplier<Rotation2d> rotationSupplier = () -> drive.getPose().getTranslation().getAngle();

    ProfiledPIDController angleController = Constants.Drive.ANGLE_PID;
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(0.1);

    // Construct command
    return Commands.run(
            () -> {
              Logger.recordOutput("Drive/Angle Target", rotationSupplier.get().getRadians());
              Logger.recordOutput(
                  "Drive/Angle Current", drive.getPose().getRotation().getRadians());

              double omega =
                  angleController.calculate(
                          drive.getRotation().getRadians(), rotationSupplier.get().getRadians())
                      + angleController.getSetpoint().velocity * Constants.Drive.ANGLE_FF;

              if (Math.abs(drive.getRotation().getRadians() - rotationSupplier.get().getRadians())
                      < Constants.Drive.ANGLE_TOL
                  && angleController.getSetpoint().velocity == 0.0) omega = 0.0;

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                      ySupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              drive.runVelocityOffset(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds, AllianceFlip.apply(drive.getRotation())),
                  Constants.ShooterConstants.location);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
