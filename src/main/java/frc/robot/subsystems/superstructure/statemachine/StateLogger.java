package frc.robot.subsystems.superstructure.statemachine;

import org.littletonrobotics.junction.Logger;

/** Common AdvantageKit logging for the reusable state machine. */
public final class StateLogger {
  private StateLogger() {}

  public static <S extends Enum<S>> void log(String prefix, StateMachine<S> machine) {
    Logger.recordOutput(prefix + "/CurrentState", machine.getCurrentState().name());
    Logger.recordOutput(prefix + "/WantedState", machine.getWantedState().name());
    Logger.recordOutput(prefix + "/PreviousState", machine.getPreviousState().name());
    Logger.recordOutput(prefix + "/TransitionReason", machine.getTransitionReason());
    Logger.recordOutput(prefix + "/TimeInState", machine.getTimeInState());
    Logger.recordOutput(prefix + "/TransitionAccepted", machine.getLastTransition().accepted());
  }
}
