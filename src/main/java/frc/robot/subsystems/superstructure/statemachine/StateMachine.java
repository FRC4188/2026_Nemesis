package frc.robot.subsystems.superstructure.statemachine;

import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;

/** Base for lightweight, typed state machines with request, transition, and lifecycle hooks. */
public class StateMachine<S extends Enum<S>> {
  private final String logPrefix;
  private S currentState;
  private S previousState;
  private S wantedState;
  private double stateEntryTimestamp;
  private String transitionReason = "Initialized";
  private StateTransition<S> lastTransition;

  public StateMachine(String logPrefix, S initialState) {
    this.logPrefix = Objects.requireNonNull(logPrefix);
    currentState = Objects.requireNonNull(initialState);
    previousState = initialState;
    wantedState = initialState;
    stateEntryTimestamp = Timer.getFPGATimestamp();
    lastTransition =
        new StateTransition<>(
            initialState, initialState, initialState, true, transitionReason, stateEntryTimestamp);
  }

  public final void requestState(S requestedState) {
    wantedState = Objects.requireNonNull(requestedState);
  }

  public final boolean transitionTo(S nextState, String reason) {
    Objects.requireNonNull(nextState);
    Objects.requireNonNull(reason);
    double now = Timer.getFPGATimestamp();

    if (nextState == currentState) {
      transitionReason = reason;
      lastTransition =
          new StateTransition<>(currentState, nextState, currentState, true, reason, now);
      return true;
    }

    if (!canTransitionTo(nextState)) {
      transitionReason = "Rejected: " + reason;
      lastTransition =
          new StateTransition<>(currentState, nextState, currentState, false, reason, now);
      onTransitionRejected(nextState, reason);
      return false;
    }

    S oldState = currentState;
    onExit(oldState, nextState);
    previousState = oldState;
    currentState = nextState;
    stateEntryTimestamp = now;
    transitionReason = reason;
    lastTransition =
        new StateTransition<>(oldState, nextState, currentState, true, reason, stateEntryTimestamp);
    onEnter(currentState, oldState);
    return true;
  }

  /** Runs state-specific behavior and common logging once per robot loop. */
  public final void periodic() {
    runStateMachine();
    StateLogger.log(logPrefix, this);
    logAdditionalOutputs();
  }

  protected boolean canTransitionTo(S nextState) {
    return true;
  }

  protected void onEnter(S state, S previousState) {}

  protected void onExit(S state, S nextState) {}

  protected void onTransitionRejected(S requestedState, String reason) {}

  protected void runStateMachine() {}

  protected void logAdditionalOutputs() {}

  public final S getCurrentState() {
    return currentState;
  }

  public final S getPreviousState() {
    return previousState;
  }

  public final S getWantedState() {
    return wantedState;
  }

  public final double getStateEntryTimestamp() {
    return stateEntryTimestamp;
  }

  public final double getTimeInState() {
    return Timer.getFPGATimestamp() - stateEntryTimestamp;
  }

  public final String getTransitionReason() {
    return transitionReason;
  }

  public final StateTransition<S> getLastTransition() {
    return lastTransition;
  }
}
