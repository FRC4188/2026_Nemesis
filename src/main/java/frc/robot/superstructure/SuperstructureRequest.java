package frc.robot.superstructure;

import frc.robot.statemachine.StateRequest;

/** Immutable typed request sent by command wrappers to the Superstructure. */
public final class SuperstructureRequest
    implements StateRequest<SuperstructureState, SuperstructureGoal> {
  public final SuperstructureState wantedState;
  public final SuperstructureGoal goal;
  public final boolean held;
  public final boolean allowInterrupt;
  public final boolean fromAuto;

  public SuperstructureRequest(
      SuperstructureState wantedState,
      SuperstructureGoal goal,
      boolean held,
      boolean allowInterrupt,
      boolean fromAuto) {
    this.wantedState = wantedState;
    this.goal = goal;
    this.held = held;
    this.allowInterrupt = allowInterrupt;
    this.fromAuto = fromAuto;
  }

  public static SuperstructureRequest idle() {
    return request(SuperstructureState.IDLE, SuperstructureGoal.none());
  }

  public static SuperstructureRequest intake() {
    return intake(8.75);
  }

  public static SuperstructureRequest intake(double volts) {
    return request(
        SuperstructureState.INTAKING,
        new SuperstructureGoal(0.0, 0.0, volts, 0.0, 0.0, false, false));
  }

  public static SuperstructureRequest eject() {
    return request(SuperstructureState.EJECTING, SuperstructureGoal.none());
  }

  public static SuperstructureRequest staticAim() {
    return request(SuperstructureState.STATIC_AIMING, SuperstructureGoal.none());
  }

  public static SuperstructureRequest staticShoot() {
    return request(SuperstructureState.STATIC_SHOOTING, SuperstructureGoal.none());
  }

  public static SuperstructureRequest manualAim(double distanceMeters) {
    return request(
        SuperstructureState.MANUAL_AIMING,
        new SuperstructureGoal(distanceMeters, 0.0, 0.0, 0.0, 0.0, true, false));
  }

  public static SuperstructureRequest manualShoot(double distanceMeters) {
    return request(
        SuperstructureState.MANUAL_SHOOTING,
        new SuperstructureGoal(distanceMeters, 0.0, 0.0, 0.0, 0.0, true, false));
  }

  public static SuperstructureRequest passAim() {
    return request(SuperstructureState.PASS_AIMING, SuperstructureGoal.none());
  }

  public static SuperstructureRequest passShoot() {
    return request(SuperstructureState.PASS_SHOOTING, SuperstructureGoal.none());
  }

  public static SuperstructureRequest dataShoot(double rpm) {
    return request(
        SuperstructureState.TUNING_SHOOTING,
        new SuperstructureGoal(0.0, 0.0, 0.0, rpm, 0.0, false, false));
  }

  public static SuperstructureRequest wristManual(double volts) {
    return request(
        SuperstructureState.WRIST_MANUAL,
        new SuperstructureGoal(0.0, volts, 0.0, 0.0, 0.0, false, false));
  }

  public static SuperstructureRequest wristSlowUp(double delaySeconds) {
    return request(
        SuperstructureState.WRIST_SLOW_UP,
        new SuperstructureGoal(0.0, 0.0, 0.0, 0.0, delaySeconds, false, false));
  }

  public static SuperstructureRequest wristDownNoStall() {
    return request(SuperstructureState.WRIST_DOWN_NO_STALL, SuperstructureGoal.none());
  }

  public static SuperstructureRequest wristGoodStow() {
    return request(SuperstructureState.WRIST_GOOD_STOW, SuperstructureGoal.none());
  }

  public static SuperstructureRequest wristForceDown() {
    return request(SuperstructureState.WRIST_FORCE_DOWN, SuperstructureGoal.none());
  }

  public SuperstructureRequest asAuto() {
    return new SuperstructureRequest(wantedState, goal.asAuto(), held, allowInterrupt, true);
  }

  private static SuperstructureRequest request(SuperstructureState state, SuperstructureGoal goal) {
    return new SuperstructureRequest(state, goal, true, true, false);
  }

  @Override
  public SuperstructureState wantedState() {
    return wantedState;
  }

  @Override
  public SuperstructureGoal goal() {
    return goal;
  }

  @Override
  public boolean held() {
    return held;
  }

  @Override
  public boolean allowInterrupt() {
    return allowInterrupt;
  }
}
