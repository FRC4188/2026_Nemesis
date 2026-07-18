package frc.robot.drivecontrol;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldConstants;
import org.junit.jupiter.api.Test;

class DriveRequestTest {
  @Test
  void distinguishesOpenLoopAndHeadingLockRequests() {
    DriveRequest openLoop = DriveRequest.teleopOpenLoop(0.25, -0.5, 0.75);
    DriveRequest heading =
        DriveRequest.teleopHeadingLock(0.25, -0.5, HeadingTargetType.SIDE_90, Rotation2d.kCW_90deg);

    assertEquals(DriveOwner.TELEOP, openLoop.owner);
    assertEquals(DriveControlState.TELEOP_OPEN_LOOP, openLoop.wantedState);
    assertFalse(openLoop.headingLock);
    assertEquals(0.75, openLoop.omegaPercent, 1e-9);

    assertEquals(DriveControlState.TELEOP_HEADING_LOCK, heading.wantedState);
    assertTrue(heading.headingLock);
    assertEquals(HeadingTargetType.SIDE_90, heading.headingTargetType);
    assertEquals(Rotation2d.kCW_90deg, heading.explicitHeadingTarget);
  }

  @Test
  void idleRequestHasNoOwnerOrDriveInput() {
    DriveRequest idle = DriveRequest.idle();

    assertEquals(DriveOwner.NONE, idle.owner);
    assertEquals(DriveControlState.IDLE_X, idle.wantedState);
    assertFalse(idle.hasDriveInput);
  }

  @Test
  void headingButtonsKeepXThenAThenHubPriority() {
    DriveController controller = DriveController.getInstance();
    Pose2d pose =
        new Pose2d(
            new Translation2d(3.0, FieldConstants.field_center.getY() + 0.5), Rotation2d.kZero);

    controller.acceptTeleopInput(0.0, 0.0, 0.0, true, true, true, pose);
    assertEquals(HeadingTargetType.SIDE_90, controller.getActiveRequest().headingTargetType);

    controller.acceptTeleopInput(0.0, 0.0, 0.0, true, true, false, pose);
    assertEquals(HeadingTargetType.DEPOT, controller.getActiveRequest().headingTargetType);

    controller.acceptTeleopInput(0.0, 0.0, 0.0, true, false, false, pose);
    assertEquals(HeadingTargetType.HUB, controller.getActiveRequest().headingTargetType);
  }
}
