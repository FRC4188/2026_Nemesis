package frc.robot.statemachine;

/** Typed external intent supplied to a state machine. */
public interface StateRequest<S extends Enum<S>, G> {
  S wantedState();

  G goal();

  boolean held();

  boolean allowInterrupt();
}
