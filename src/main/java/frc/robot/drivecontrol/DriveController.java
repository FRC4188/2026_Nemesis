package frc.robot.drivecontrol;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Coordinates migrated teleop drive behavior while auto and characterization remain commands. */
public final class DriveController extends StateMachine<DriveControlState> {
  private static DriveController instance;

  public static synchronized DriveController getInstance() {
    if (instance == null) {
      instance = new DriveController();
    }
    return instance;
  }

  private final Drive drive = Drive.getInstance();
  private final ProfiledPIDController angleController = Constants.DriveConstants.ANGLE_PID;

  private DriveRequest activeRequest = DriveRequest.idle();
  private DriveOwner owner = DriveOwner.NONE;
  private DriveTransitionReason transitionReason = DriveTransitionReason.INITIALIZED;
  private String rejectedRequest = "";
  private String rejectReason = "";
  private Rotation2d headingTarget = Rotation2d.kZero;
  private double outputVx;
  private double outputVy;
  private double outputOmega;
  private boolean acceptVision = true;

  private DriveController() {
    super("DriveController", DriveControlState.DISABLED);
  }

  public void acceptTeleopInput(
      double xPercent,
      double yPercent,
      double omegaPercent,
      boolean rightBumperHeld,
      boolean aHeld,
      boolean xHeld,
      Pose2d currentPose) {
    if (xHeld) {
      headingTarget =
          currentPose.getTranslation().getY() > FieldConstants.field_center.getY()
              ? Rotation2d.kCW_90deg
              : Rotation2d.kCCW_90deg;
      activeRequest =
          DriveRequest.teleopHeadingLock(
              xPercent, yPercent, HeadingTargetType.SIDE_90, headingTarget);
    } else if (aHeld) {
      Translation2d nearestDepotCorner =
          currentPose
              .getTranslation()
              .nearest(
                  List.of(
                      AllianceFlip.apply(FieldConstants.Depot.left_far_corner),
                      AllianceFlip.apply(
                          new Translation2d(
                              FieldConstants.Depot.left_far_corner.getX(),
                              FieldConstants.field_width
                                  - FieldConstants.Depot.left_far_corner.getY()))));
      headingTarget = nearestDepotCorner.minus(currentPose.getTranslation()).getAngle();
      activeRequest =
          DriveRequest.teleopHeadingLock(
              xPercent, yPercent, HeadingTargetType.DEPOT, headingTarget);
    } else if (rightBumperHeld) {
      headingTarget =
          AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
              .minus(currentPose.getTranslation())
              .getAngle();
      activeRequest =
          DriveRequest.teleopHeadingLock(xPercent, yPercent, HeadingTargetType.HUB, headingTarget);
    } else {
      headingTarget = Rotation2d.kZero;
      activeRequest = DriveRequest.teleopOpenLoop(xPercent, yPercent, omegaPercent);
    }
  }

  /** Stores a future auto-path request; command-based path execution is intentionally unchanged. */
  public void requestAutoPath(DriveGoal goal) {
    activeRequest = DriveRequest.autoPath(goal);
  }

  public void requestCharacterization(double volts) {
    activeRequest = DriveRequest.characterization(volts);
  }

  public void cancelAuto() {
    if (activeRequest.owner == DriveOwner.AUTO) {
      activeRequest = DriveRequest.idle();
    }
    transitionReason = DriveTransitionReason.AUTO_CANCELLED;
  }

  public void requestIdle() {
    activeRequest = DriveRequest.idle();
  }

  public DriveOwner getOwner() {
    return owner;
  }

  public DriveRequest getActiveRequest() {
    return activeRequest;
  }

  public boolean isTeleopDriving() {
    return owner == DriveOwner.TELEOP;
  }

  public boolean isAutoDriving() {
    return owner == DriveOwner.AUTO;
  }

  public boolean isHeadingLocked() {
    return getCurrentState() == DriveControlState.TELEOP_HEADING_LOCK;
  }

  @Override
  protected void runStateMachine() {
    if (DriverStation.isDisabled()) {
      owner = DriveOwner.NONE;
      requestState(DriveControlState.DISABLED);
      if (getCurrentState() != DriveControlState.DISABLED) {
        transition(DriveControlState.DISABLED, DriveTransitionReason.DISABLED);
      }
      runCurrentState();
      return;
    }

    DriveControlState requestedState = convertRequestToState(activeRequest);
    if (!requestAllowed(activeRequest)) {
      rejectedRequest = activeRequest.wantedState.name();
      rejectReason = rejectionReason(activeRequest);
      requestedState = DriveControlState.IDLE_X;
      owner = DriveOwner.NONE;
      transitionReason = DriveTransitionReason.REJECTED;
    } else {
      rejectedRequest = "";
      rejectReason = "";
      owner = activeRequest.owner;
    }

    requestState(requestedState);
    if (getCurrentState() != requestedState) {
      transition(
          requestedState,
          transitionReason == DriveTransitionReason.REJECTED
              ? DriveTransitionReason.REJECTED
              : DriveTransitionReason.REQUESTED);
    }
    runCurrentState();
  }

  private DriveControlState convertRequestToState(DriveRequest request) {
    if (request.owner == DriveOwner.NONE || !request.hasDriveInput) {
      return DriveControlState.IDLE_X;
    }
    if (request.owner == DriveOwner.CHARACTERIZATION) {
      return DriveControlState.CHARACTERIZATION;
    }
    if (request.owner == DriveOwner.AUTO) {
      return DriveControlState.AUTO_PATH_FOLLOWING;
    }
    return request.headingLock
        ? DriveControlState.TELEOP_HEADING_LOCK
        : DriveControlState.TELEOP_OPEN_LOOP;
  }

  private boolean requestAllowed(DriveRequest request) {
    return switch (request.owner) {
      case NONE -> true;
      case TELEOP -> DriverStation.isTeleop();
      case AUTO -> DriverStation.isAutonomous();
      case CHARACTERIZATION -> true;
    };
  }

  private String rejectionReason(DriveRequest request) {
    return switch (request.owner) {
      case TELEOP -> "Teleop request outside teleop";
      case AUTO -> "Auto request outside autonomous";
      case CHARACTERIZATION -> "Characterization request rejected";
      case NONE -> "";
    };
  }

  private void transition(DriveControlState next, DriveTransitionReason reason) {
    transitionReason = reason;
    if (!transitionTo(next, reason.name())) {
      rejectedRequest = next.name();
      rejectReason = "Transition rejected";
      transitionReason = DriveTransitionReason.REJECTED;
    }
  }

  @Override
  protected boolean canTransitionTo(DriveControlState next) {
    return getCurrentState() != DriveControlState.FAULTED
        || next == DriveControlState.FAULTED
        || next == DriveControlState.IDLE_X;
  }

  @Override
  protected void onEnter(DriveControlState state, DriveControlState previousState) {
    if (state == DriveControlState.TELEOP_HEADING_LOCK) {
      angleController.reset(
          drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond);
    }
  }

  private void runCurrentState() {
    switch (getCurrentState()) {
      case DISABLED -> {
        drive.stopWithX();
        setAcceptVision(true);
        clearOutputs();
      }
      case IDLE_X -> {
        // In autonomous, no DriveController request means command-based paths retain ownership.
        if (!DriverStation.isAutonomous()) {
          drive.stopWithX();
          setAcceptVision(true);
          clearOutputs();
        }
      }
      case TELEOP_OPEN_LOOP -> runOpenLoop();
      case TELEOP_HEADING_LOCK -> runHeadingLock();
      case CHARACTERIZATION -> {
        drive.runCharacterization(activeRequest.goal.characterizationVolts);
        outputVx = 0.0;
        outputVy = 0.0;
        outputOmega = 0.0;
      }
      case AUTO_PATH_FOLLOWING, AUTO_ALIGN -> {
        // Placeholder only. Existing auto commands continue to own drive outputs.
      }
      case FAULTED -> {
        drive.stopWithX();
        setAcceptVision(true);
        clearOutputs();
      }
    }
  }

  private void runOpenLoop() {
    setAcceptVision(true);
    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            activeRequest.xPercent * Constants.DriveConstants.DRIVE_MAXVEL,
            activeRequest.yPercent * Constants.DriveConstants.DRIVE_MAXVEL,
            activeRequest.omegaPercent * Constants.DriveConstants.ANGLE_MAXVEL);
    runFieldRelative(fieldRelative);
  }

  private void runHeadingLock() {
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(),
            activeRequest
                .explicitHeadingTarget
                .minus(Constants.DriveConstants.local_offset)
                .getRadians());

    if (angleController.atGoal()) {
      if (activeRequest.xPercent == 0.0 && activeRequest.yPercent == 0.0) {
        drive.stopWithX();
        setAcceptVision(false);
        clearOutputs();
        return;
      }
      omega = 0.0;
      setAcceptVision(false);
    } else {
      setAcceptVision(true);
    }

    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            activeRequest.xPercent * Constants.DriveConstants.DRIVE_MAXVEL,
            activeRequest.yPercent * Constants.DriveConstants.DRIVE_MAXVEL,
            omega);
    runFieldRelative(fieldRelative);
  }

  private void runFieldRelative(ChassisSpeeds fieldRelative) {
    ChassisSpeeds robotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative, AllianceFlip.apply(drive.getRotation()));
    outputVx = robotRelative.vxMetersPerSecond;
    outputVy = robotRelative.vyMetersPerSecond;
    outputOmega = robotRelative.omegaRadiansPerSecond;
    drive.runVelocity(robotRelative);
  }

  private void setAcceptVision(boolean accept) {
    acceptVision = accept;
    drive.acceptVision(accept);
  }

  private void clearOutputs() {
    outputVx = 0.0;
    outputVy = 0.0;
    outputOmega = 0.0;
  }

  @Override
  protected void logAdditionalOutputs() {
    Logger.recordOutput("DriveController/Owner", owner.name());
    Logger.recordOutput("DriveController/TransitionReason", transitionReason.name());
    Logger.recordOutput("DriveController/RejectedRequest", rejectedRequest);
    Logger.recordOutput("DriveController/RejectReason", rejectReason);
    Logger.recordOutput(
        "DriveController/HeadingTargetType", activeRequest.headingTargetType.name());
    Logger.recordOutput(
        "DriveController/HeadingTargetRadians", activeRequest.explicitHeadingTarget.getRadians());
    Logger.recordOutput("DriveController/HeadingErrorRadians", angleController.getPositionError());
    Logger.recordOutput("DriveController/AngleControllerAtGoal", angleController.atGoal());
    Logger.recordOutput("DriveController/RequestedXPercent", activeRequest.xPercent);
    Logger.recordOutput("DriveController/RequestedYPercent", activeRequest.yPercent);
    Logger.recordOutput("DriveController/RequestedOmegaPercent", activeRequest.omegaPercent);
    Logger.recordOutput("DriveController/OutputVxMetersPerSec", outputVx);
    Logger.recordOutput("DriveController/OutputVyMetersPerSec", outputVy);
    Logger.recordOutput("DriveController/OutputOmegaRadPerSec", outputOmega);
    Logger.recordOutput("DriveController/AcceptVision", acceptVision);
  }
}
