package frc.robot.superstructure;

public enum SuperstructureTransitionReason {
  INITIALIZED,
  REQUESTED,
  REQUEST_CLEARED,
  READY_TO_FEED,
  TIMER_ELAPSED,
  SENSOR_THRESHOLD,
  INTERNAL_STEP,
  DISABLED,
  FAULTED,
  FAULT_CLEARED,
  REJECTED
}
