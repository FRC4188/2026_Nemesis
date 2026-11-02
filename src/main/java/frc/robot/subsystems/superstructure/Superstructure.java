package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.statemachine.StateMachine;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import frc.robot.util.SOTM;
import frc.robot.util.ShotCalc;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Coordinates robot-level requests; each mechanism applies its own state in its periodic loop. */
public class Superstructure extends SubsystemBase {
  public enum RobotState {
    IDLE,
    DRIVER_CONTROL,
    INTAKING,
    SCORING,
    MANUAL_CONTROL,
    AUTONOMOUS
  }

  private enum ShotMode {
    STATIC,
    PASS,
    MANUAL
  }

  private enum ShotPhase {
    AIMING,
    SPINNING_UP,
    FEEDING
  }

  public static final LoggedNetworkNumber apexHeight =
      new LoggedNetworkNumber("Aim Tuning/Apex Height Inches", Units.inchesToMeters(102));
  public static final LoggedNetworkNumber targetHeight =
      new LoggedNetworkNumber("Aim Tuning/Target Height Inches", Units.inchesToMeters(72));
  public static final LoggedNetworkNumber passHeight =
      new LoggedNetworkNumber("Aim Tuning/Pass Height Inches", Units.inchesToMeters(120));

  private final Drive drive;
  private final Hood hood;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Wrist wrist;
  private final Vision vision;

  private boolean autonomous;
  private boolean driverEnabled;
  private double driveX;
  private double driveY;
  private double driveOmega;
  private Translation2d driveGoal = Translation2d.kZero;
  private boolean aimLock;
  private boolean intakeRequested;
  private boolean scoringRequested;
  private double scoringDistanceFeet;
  private boolean intakeWhileScoring;
  private boolean autoIntakeRequested;
  private boolean autoShootRequested;
  private boolean manualWristRequested;
  private double manualWristVolts;

  private boolean scoringWasActive;
  private ShotMode shotMode;
  private ShotPhase shotPhase;
  private double shotPhaseStart;
  private boolean initialShot = true;

  private final StateMachine<RobotState> stateMachine =
      new StateMachine<>("Superstructure", RobotState.IDLE) {
        @Override
        protected void runStateMachine() {
          RobotState wanted = determineWantedState();
          if (getWantedState() != wanted) {
            requestState(wanted);
          }
          if (getCurrentState() != getWantedState()) {
            transitionTo(getWantedState(), "Inputs updated");
          }

          switch (getCurrentState()) {
            case IDLE -> {
              updateDrive();
              stopScoringMechanisms();
              intake.stop();
            }
            case DRIVER_CONTROL -> {
              updateDrive();
              stopScoringMechanisms();
              intake.stop();
            }
            case INTAKING -> {
              updateDrive();
              stopScoringMechanisms();
              intake.intakeVolts(8.75);
            }
            case SCORING -> {
              updateDrive();
              runScoring();
              intakeWhileScoring();
            }
            case MANUAL_CONTROL -> {
              updateDrive();
              stopScoringMechanisms();
              intake.stop();
            }
            case AUTONOMOUS -> {
              if (autoIntakeRequested) {
                intake.intakeVolts(8.75);
              } else if (!autoShootRequested) {
                intake.stop();
              }
              if (autoShootRequested) {
                runScoring();
                intakeWhileScoring();
              }
            }
          }

          if (getCurrentState() != RobotState.SCORING
              && !(getCurrentState() == RobotState.AUTONOMOUS && autoShootRequested)) {
            finishScoringIfNeeded();
          }
        }
      };

  public Superstructure(
      Drive drive,
      Hood hood,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      Wrist wrist,
      Vision vision) {
    this.drive = drive;
    this.hood = hood;
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.wrist = wrist;
    this.vision = vision;
  }

  private RobotState determineWantedState() {
    if (autonomous) return RobotState.AUTONOMOUS;
    if (scoringRequested) return RobotState.SCORING;
    if (manualWristRequested) return RobotState.MANUAL_CONTROL;
    if (intakeRequested) return RobotState.INTAKING;
    if (driverEnabled) return RobotState.DRIVER_CONTROL;
    return RobotState.IDLE;
  }

  private void updateDrive() {
    if (driverEnabled) {
      drive.requestDriverControl(driveX, driveY, driveOmega, driveGoal, aimLock);
    } else {
      drive.stopDriverControl();
    }
  }

  private void stopScoringMechanisms() {
    shooter.stop();
    hopper.stop();
    hood.stop();
    if (scoringWasActive) {
      finishScoringIfNeeded();
    }
  }

  private void runScoring() {
    if (!scoringWasActive) {
      beginScoring();
    }

    updateHoodTarget();
    switch (shotPhase) {
      case AIMING -> {
        shooter.setVelocityRPM(500.0);
        boolean ready = hood.atGoal();
        if (shotMode == ShotMode.STATIC) {
          ready &=
              Constants.DriveConstants.ANGLE_PID.atGoal() || drive.getTranslationalSpeed() > 1e-9;
        }
        if (ready) {
          shotPhase = ShotPhase.SPINNING_UP;
          shotPhaseStart = Timer.getFPGATimestamp();
        }
      }
      case SPINNING_UP -> {
        shooter.setVelocityRPM(getShotRPM());
        if (Timer.getFPGATimestamp() - shotPhaseStart >= 0.1 && shooter.atGoal()) {
          shotPhase = ShotPhase.FEEDING;
          shotPhaseStart = Timer.getFPGATimestamp();
        }
      }
      case FEEDING -> {
        shooter.setVelocityRPM(getShotRPM());
        double delay = shotMode == ShotMode.STATIC ? 0.5 : 0.1;
        if (Timer.getFPGATimestamp() - shotPhaseStart >= delay) {
          switch (shotMode) {
            case STATIC -> hopper.runHopper(7.0, 2000.0);
            case PASS -> hopper.runHopper(9.0, 2000.0);
            case MANUAL -> hopper.runHopper(9.0, 5000.0);
          }
          if (hopper.indexAtGoal()) {
            initialShot = false;
          }
          if (Timer.getFPGATimestamp() - shotPhaseStart >= delay + 0.1) {
            wrist.requestCompression(intakeWhileScoring);
          }
        }
      }
    }
  }

  private void beginScoring() {
    scoringWasActive = true;
    initialShot = true;
    if (scoringDistanceFeet == 0.0) {
      shotMode = isStaticShotSide() ? ShotMode.STATIC : ShotMode.PASS;
    } else {
      shotMode = ShotMode.MANUAL;
    }
    shotPhase = ShotPhase.AIMING;
    shotPhaseStart = Timer.getFPGATimestamp();
  }

  private void finishScoringIfNeeded() {
    if (!scoringWasActive) return;
    scoringWasActive = false;
    initialShot = true;
    shooter.stop();
    hopper.stop();
    hood.stop();
    intake.stop();
    wrist.requestHold();
  }

  private void updateHoodTarget() {
    switch (shotMode) {
      case STATIC -> {
        Translation2d target = AllianceFlip.apply(FieldConstants.Hub.hub_center_2d);
        double distance =
            SOTM.lookahead(target, drive.getChassisSpeeds(), SOTM.TOF_SECONDS)
                .minus(drive.getPose().getTranslation())
                .getNorm();
        hood.setAngle(
            ShotCalc.getShotAngle(distance, apexHeight.getAsDouble(), targetHeight.getAsDouble()));
      }
      case PASS -> hood.setAngle(Rotation2d.fromDegrees(40.0));
      case MANUAL -> hood.setAngle(inclineHeuristic(Units.feetToMeters(scoringDistanceFeet)));
    }
  }

  private double getShotRPM() {
    return switch (shotMode) {
      case STATIC -> {
        Translation2d target = AllianceFlip.apply(FieldConstants.Hub.hub_center_2d);
        double distance =
            SOTM.lookahead(
                    target,
                    drive.getChassisSpeeds(),
                    ShotCalc.getTimeOfFlight(apexHeight.getAsDouble(), targetHeight.getAsDouble()))
                .minus(drive.getPose().getTranslation())
                .getNorm();
        yield ShotCalc.getShotRPM(distance, apexHeight.getAsDouble(), targetHeight.getAsDouble())
            + (initialShot ? 200.0 : 0.0);
      }
      case PASS -> {
        double distanceFeet =
            Units.metersToFeet(
                AllianceFlip.apply(drive.getPose()).getX()
                    - Units.inchesToMeters(156.61)
                    + Units.feetToMeters(2.0));
        yield ShotCalc.getShotRPM(
            distanceFeet,
            Units.inchesToMeters(passHeight.getAsDouble()),
            Units.inchesToMeters(0.0));
      }
      case MANUAL -> {
        double distance = Units.feetToMeters(scoringDistanceFeet);
        yield regressionRPM(distance)
            + (initialShot ? 300.0 - hopper.getIndexerSpeed() * 300.0 / 5000.0 : 0.0);
      }
    };
  }

  private boolean isStaticShotSide() {
    double hubLine = AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX();
    double robotX = drive.getPose().getX();
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue
        ? robotX <= hubLine
        : robotX >= hubLine;
  }

  private static Rotation2d inclineHeuristic(double distanceMeters) {
    return Rotation2d.fromRadians(Math.PI / 2.0 - Math.atan(7.0 / distanceMeters));
  }

  public static double regressionRPM(double distanceMeters) {
    return 38.0 * Math.pow(distanceMeters - 1.5, 2.0) + 1800.0;
  }

  public static Rotation2d regressionAngle(double distanceMeters) {
    return inclineHeuristic(distanceMeters);
  }

  private void intakeWhileScoring() {
    if (intakeWhileScoring) {
      intake.intakeVolts(8.75);
      wrist.requestCompression(true);
    } else if (wrist.isCompressionReady()) {
      intake.intakeVolts(5.0);
      wrist.requestCompression(false);
    } else {
      intake.stop();
      if (shotPhase == ShotPhase.FEEDING) {
        wrist.requestCompression(false);
      }
    }
  }

  public void updateTeleopInputs(
      boolean driverEnabled,
      double x,
      double y,
      double omega,
      Translation2d goal,
      boolean aimLock,
      boolean intakeRequested,
      boolean scoringRequested,
      double scoringDistanceFeet,
      boolean intakeWhileScoring,
      boolean manualWristRequested) {
    this.driverEnabled = driverEnabled;
    driveX = x;
    driveY = y;
    driveOmega = omega;
    driveGoal = goal;
    this.aimLock = aimLock;
    this.intakeRequested = intakeRequested;
    this.scoringRequested = scoringRequested;
    this.scoringDistanceFeet = scoringDistanceFeet;
    this.intakeWhileScoring = intakeWhileScoring;
    this.manualWristRequested = manualWristRequested;
  }

  public void setAutonomous(boolean enabled) {
    autonomous = enabled;
  }

  public void setAutoIntake(boolean enabled) {
    autoIntakeRequested = enabled;
    if (!enabled && !autoShootRequested) intake.stop();
  }

  public void setAutoShoot(boolean enabled) {
    if (enabled) {
      autoIntakeRequested = false;
      intake.stop();
      wrist.stop();
      drive.requestAutonomousAim(AllianceFlip.apply(FieldConstants.Hub.hub_center_2d), true);
    } else {
      drive.requestAutonomousAim(AllianceFlip.apply(FieldConstants.Hub.hub_center_2d), false);
    }
    autoShootRequested = enabled;
    if (!enabled) finishScoringIfNeeded();
  }

  public void setVisionEnabled(boolean enabled) {
    vision.enableVision(enabled);
  }

  public boolean isVisionEnabled() {
    return vision.isVisionEnabled();
  }

  public double getRegressionRPM() {
    Translation2d target = AllianceFlip.apply(FieldConstants.Hub.hub_center_2d);
    return regressionRPM(target.getDistance(drive.getPose().getTranslation()));
  }

  public double getRegressionAngleDegrees() {
    Translation2d target = AllianceFlip.apply(FieldConstants.Hub.hub_center_2d);
    return regressionAngle(target.getDistance(drive.getPose().getTranslation())).getDegrees();
  }

  public RobotState getWantedRobotState() {
    return stateMachine.getWantedState();
  }

  public RobotState getCurrentRobotState() {
    return stateMachine.getCurrentState();
  }

  @Override
  public void periodic() {
    stateMachine.periodic();
  }
}
