package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class SOTM { // Experimental Class for Offseason
  private static final Drive drive = Drive.getInstance();
  public static final double TOF_SECONDS = 1.03702895;

  private static final double SOTM_LOOKAHEAD_SECONDS = 0.02;

  private static final double SOTM_MAX_SHOOTER_VELOCITY_MPS =
      (ShotCalc.kMaxRPM + 181.26888) / 302.1148;

  private static final double SOTM_MAX_HOOD_VELOCITY_RAD_PER_SEC =
      (100.0 / Constants.HoodConstants.kGearRatio) * 2.0 * Math.PI;

  public static final Rotation2d kMaxPolarVelocity = Rotation2d.fromRadians(0.4);

  public static Translation2d lookahead(
      Translation2d target, ChassisSpeeds currentSpeeds, double flightTime) {

    ChassisSpeeds currentFCSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, drive.getRotation());

    double displaceX;
    double displaceY;

    displaceX = currentFCSpeeds.vxMetersPerSecond * flightTime;
    displaceY = currentFCSpeeds.vyMetersPerSecond * flightTime;

    return target.minus(new Translation2d(displaceX, displaceY));
  }

  public static double getSOTMMaxVelocity(
      Translation2d desiredFieldRelativeVelocity, Translation2d robotToTarget) {

    double desiredVelocityMagnitude = desiredFieldRelativeVelocity.getNorm();

    if (desiredVelocityMagnitude <= 1e-9) {
      return Constants.DriveConstants.DRIVE_MAXVEL;
    }

    double robotTargetDistance = robotToTarget.getNorm();

    if (robotTargetDistance <= 1e-9) {
      return Constants.DriveConstants.DRIVE_MAXVEL;
    }

    double robotAngle =
        Math.abs(
            robotToTarget.getAngle().minus(desiredFieldRelativeVelocity.getAngle()).getRadians());

    double hubAngle = kMaxPolarVelocity.getRadians() * TOF_SECONDS;

    double lookaheadAngle = Math.PI - robotAngle - hubAngle;

    double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;

    if (lookaheadAngle > 0.0) {
      double robotLookaheadDistance =
          robotTargetDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);

      maxLinearVelocityMagnitude = robotLookaheadDistance / SOTM.TOF_SECONDS;
    }

    return Math.min(Constants.DriveConstants.DRIVE_MAXVEL, maxLinearVelocityMagnitude);
  }

  public static double getSOTMMaxVelocityExtended(
      Translation2d desiredFieldRelativeVelocity, Translation2d robotToTarget) {

    final double desiredVelocityMagnitude = desiredFieldRelativeVelocity.getNorm();

    if (desiredVelocityMagnitude <= 1e-9) {
      return Constants.DriveConstants.DRIVE_MAXVEL;
    }

    final double robotTargetDistance = robotToTarget.getNorm();

    if (robotTargetDistance <= 1e-9) {
      return Constants.DriveConstants.DRIVE_MAXVEL;
    }

    // Polar angular velocity constraint
    final double robotAngle =
        Math.abs(
            robotToTarget.getAngle().minus(desiredFieldRelativeVelocity.getAngle()).getRadians());

    final double hubAngle = kMaxPolarVelocity.getRadians() * SOTM.TOF_SECONDS;

    final double lookaheadAngle = Math.PI - robotAngle - hubAngle;

    double maxVelocity = Constants.DriveConstants.DRIVE_MAXVEL;

    if (lookaheadAngle > 0.0) {
      final double robotLookaheadDistance =
          robotTargetDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);

      maxVelocity = Math.min(maxVelocity, robotLookaheadDistance / SOTM.TOF_SECONDS);
    }

    // Shooter / hood constraint
    final Translation2d velocityDirection =
        desiredFieldRelativeVelocity.div(desiredVelocityMagnitude);

    for (double candidateVelocity = maxVelocity;
        candidateVelocity >= 0.0;
        candidateVelocity -= 0.05) {

      final Translation2d candidateRobotVelocity = velocityDirection.times(candidateVelocity);

      // Current shot
      final double currentShotVelocity = ShotCalc.getShotMPS(robotTargetDistance);

      final double currentShotAngle = ShotCalc.getShotAngle(robotTargetDistance).getRadians();

      final Translation2d targetDirection = robotToTarget.div(robotTargetDistance);

      final Translation2d lateralDirection =
          new Translation2d(-targetDirection.getY(), targetDirection.getX());

      final double velocityTowardTarget = candidateRobotVelocity.dot(targetDirection);

      final double velocityLateral = candidateRobotVelocity.dot(lateralDirection);

      final double shotHorizontalVelocity = currentShotVelocity * Math.cos(currentShotAngle);

      final double shotVerticalVelocity = currentShotVelocity * Math.sin(currentShotAngle);
      final double requiredHorizontalVelocity =
          Math.hypot(shotHorizontalVelocity - velocityTowardTarget, velocityLateral);

      final double requiredShooterVelocity =
          Math.hypot(requiredHorizontalVelocity, shotVerticalVelocity);

      if (requiredShooterVelocity > SOTM_MAX_SHOOTER_VELOCITY_MPS) {
        continue;
      }

      final double requiredHoodAngle = Math.atan2(shotVerticalVelocity, requiredHorizontalVelocity);

      if (requiredHoodAngle < Constants.HoodConstants.Min_A.getRadians()
          || requiredHoodAngle > Constants.HoodConstants.Max_A.getRadians()) {
        continue;
      }

      // Future shot state
      final Translation2d futureRobotToTarget =
          robotToTarget.minus(candidateRobotVelocity.times(SOTM_LOOKAHEAD_SECONDS));

      final double futureDistance = futureRobotToTarget.getNorm();

      if (futureDistance <= 1e-9) {
        continue;
      }

      final double futureShotVelocity = ShotCalc.getShotMPS(futureDistance);

      final double futureShotAngle = ShotCalc.getShotAngle(futureDistance).getRadians();

      final Translation2d futureTargetDirection = futureRobotToTarget.div(futureDistance);

      final Translation2d futureLateralDirection =
          new Translation2d(-futureTargetDirection.getY(), futureTargetDirection.getX());

      final double futureVelocityTowardTarget = candidateRobotVelocity.dot(futureTargetDirection);

      final double futureVelocityLateral = candidateRobotVelocity.dot(futureLateralDirection);

      final double futureShotHorizontalVelocity = futureShotVelocity * Math.cos(futureShotAngle);

      final double futureShotVerticalVelocity = futureShotVelocity * Math.sin(futureShotAngle);

      final double futureRequiredHorizontalVelocity =
          Math.hypot(
              futureShotHorizontalVelocity - futureVelocityTowardTarget, futureVelocityLateral);

      final double futureRequiredShooterVelocity =
          Math.hypot(futureRequiredHorizontalVelocity, futureShotVerticalVelocity);

      if (futureRequiredShooterVelocity > SOTM_MAX_SHOOTER_VELOCITY_MPS) {
        continue;
      }

      final double futureRequiredHoodAngle =
          Math.atan2(futureShotVerticalVelocity, futureRequiredHorizontalVelocity);

      if (futureRequiredHoodAngle < Constants.HoodConstants.Min_A.getRadians()
          || futureRequiredHoodAngle > Constants.HoodConstants.Max_A.getRadians()) {
        continue;
      }
      
      final double requiredHoodVelocity =
          Math.abs(futureRequiredHoodAngle - requiredHoodAngle) / SOTM_LOOKAHEAD_SECONDS;

      if (requiredHoodVelocity > SOTM_MAX_HOOD_VELOCITY_RAD_PER_SEC) {
        continue;
      }

      return candidateVelocity;
    }

    return 0.0;
  }
}
