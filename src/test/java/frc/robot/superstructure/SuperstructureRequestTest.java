package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SuperstructureRequestTest {
  @Test
  void preservesTypedManualGoalsAndAutoOrigin() {
    SuperstructureRequest request = SuperstructureRequest.manualShoot(3.25).asAuto();

    assertEquals(SuperstructureState.MANUAL_SHOOTING, request.wantedState);
    assertEquals(3.25, request.goal.manualDistanceMeters, 1e-9);
    assertTrue(request.goal.useManualDistance);
    assertTrue(request.fromAuto);
    assertTrue(request.goal.fromAuto);
    assertTrue(request.held);
    assertTrue(request.allowInterrupt);
  }

  @Test
  void keepsAimAndShootAsDistinctIntent() {
    assertEquals(SuperstructureState.STATIC_AIMING, SuperstructureRequest.staticAim().wantedState);
    assertEquals(
        SuperstructureState.STATIC_SHOOTING, SuperstructureRequest.staticShoot().wantedState);
    assertFalse(SuperstructureRequest.staticAim().goal.useManualDistance);
  }

  @Test
  void carriesExactIntakeVoltage() {
    SuperstructureRequest request = SuperstructureRequest.intake(7.5).asAuto();

    assertEquals(SuperstructureState.INTAKING, request.wantedState);
    assertEquals(7.5, request.goal.intakeVolts, 1e-9);
    assertTrue(request.fromAuto);
  }

  @Test
  void carriesTuningRpmAndSlowUpDelay() {
    SuperstructureRequest tuning = SuperstructureRequest.dataShoot(2450.0);
    SuperstructureRequest slowUp = SuperstructureRequest.wristSlowUp(1.5);

    assertEquals(SuperstructureState.TUNING_SHOOTING, tuning.wantedState);
    assertEquals(2450.0, tuning.goal.shooterRPM, 1e-9);
    assertEquals(SuperstructureState.WRIST_SLOW_UP, slowUp.wantedState);
    assertEquals(1.5, slowUp.goal.delaySeconds, 1e-9);
  }
}
