package frc.robot.subsystems.superstructure.statemachine;

/** Immutable record of the most recent accepted or rejected state transition. */
public record StateTransition<S extends Enum<S>>(
    S previousState,
    S requestedState,
    S resultingState,
    boolean accepted,
    String reason,
    double timestampSeconds) {}
