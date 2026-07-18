package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.statemachine.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Coordinates scoring mechanisms from typed requests while commands retain lifecycle ownership. */
public final class Superstructure extends StateMachine<SuperstructureState> {
  private record RequestSlot(SuperstructureRequest request, long order) {}

  private static Superstructure instance;

  public static synchronized Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  private final Drive drive = Drive.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Hopper hopper = Hopper.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final Hood hood = Hood.getInstance();
  private final Wrist wrist = Wrist.getInstance();
  private final SuperstructureInputs inputs = new SuperstructureInputs();
  private final Map<Object, RequestSlot> requests = new IdentityHashMap<>();
  private final Set<Object> completedOwners = Collections.newSetFromMap(new IdentityHashMap<>());

  private long nextRequestOrder;
  private Object activeOwner;
  private Object auxiliaryAimOwner;
  private Object previousAuxiliaryAimOwner;
  private SuperstructureRequest auxiliaryAimRequest = SuperstructureRequest.idle();
  private Object auxiliaryIntakeOwner;
  private SuperstructureRequest auxiliaryIntakeRequest = SuperstructureRequest.idle();
  private Object previousAuxiliaryIntakeOwner;
  private Object auxiliaryWristOwner;
  private Object executingWristOwner;
  private SuperstructureRequest auxiliaryWristRequest = SuperstructureRequest.idle();
  private SuperstructureState auxiliaryWristState = SuperstructureState.IDLE;
  private double auxiliaryWristEntryTimestamp;
  private SuperstructureRequest activeRequest = SuperstructureRequest.idle();
  private SuperstructureGoal activeGoal = SuperstructureGoal.none();
  private SuperstructureTransitionReason transitionReason =
      SuperstructureTransitionReason.INITIALIZED;
  private boolean initialShots = true;
  private String rejectedWantedState = "";
  private String rejectReason = "";
  private double goalDistanceMeters;
  private double goalRPM;
  private double goalHoodAngleDegrees;

  private Superstructure() {
    super("Superstructure", SuperstructureState.IDLE);
  }

  public synchronized void request(Object owner, SuperstructureRequest request) {
    RequestSlot existing = requests.get(owner);
    long order = existing == null ? nextRequestOrder++ : existing.order();
    requests.put(owner, new RequestSlot(request, order));
    if (existing == null) {
      completedOwners.remove(owner);
    }
  }

  public synchronized void clearRequest(Object owner) {
    requests.remove(owner);
    completedOwners.remove(owner);
    if (owner == activeOwner) {
      activeOwner = null;
    }
  }

  public synchronized void clearAllRequests() {
    if (auxiliaryWristState != SuperstructureState.IDLE) {
      wrist.stop();
    }
    if (auxiliaryIntakeOwner != null) {
      intake.stop();
    }
    if (auxiliaryAimOwner != null) {
      hood.stop();
    }
    requests.clear();
    completedOwners.clear();
    activeOwner = null;
    auxiliaryAimOwner = null;
    previousAuxiliaryAimOwner = null;
    auxiliaryAimRequest = SuperstructureRequest.idle();
    auxiliaryIntakeOwner = null;
    auxiliaryIntakeRequest = SuperstructureRequest.idle();
    previousAuxiliaryIntakeOwner = null;
    auxiliaryWristOwner = null;
    executingWristOwner = null;
    auxiliaryWristRequest = SuperstructureRequest.idle();
    auxiliaryWristState = SuperstructureState.IDLE;
    activeRequest = SuperstructureRequest.idle();
    activeGoal = SuperstructureGoal.none();
  }

  public synchronized boolean isRequestComplete(Object owner) {
    return completedOwners.contains(owner);
  }

  public boolean getInitialShots() {
    return initialShots;
  }

  public SuperstructureRequest getActiveRequest() {
    return activeRequest;
  }

  @Override
  protected void runStateMachine() {
    inputs.update(shooter, hopper, wrist);

    if (DriverStation.isDisabled()) {
      requestState(SuperstructureState.IDLE);
      activeRequest = SuperstructureRequest.idle();
      activeGoal = SuperstructureGoal.none();
      if (getCurrentState() != SuperstructureState.IDLE) {
        transition(SuperstructureState.IDLE, SuperstructureTransitionReason.DISABLED);
      }
      stopAll();
      return;
    }

    if (inputs.faulted) {
      requestState(SuperstructureState.FAULTED);
      if (getCurrentState() != SuperstructureState.FAULTED) {
        transition(SuperstructureState.FAULTED, SuperstructureTransitionReason.FAULTED);
      }
      stopAll();
      return;
    }

    selectActiveRequest();
    reconcileWantedState();
    runCurrentState();
    runAuxiliaryRequests();
  }

  private synchronized void selectActiveRequest() {
    Object selectedOwner = null;
    RequestSlot selectedSlot = null;
    Object selectedAimOwner = null;
    RequestSlot selectedAimSlot = null;
    Object selectedIntakeOwner = null;
    RequestSlot selectedIntakeSlot = null;
    Object selectedWristOwner = null;
    RequestSlot selectedWristSlot = null;

    for (Map.Entry<Object, RequestSlot> entry : requests.entrySet()) {
      if (completedOwners.contains(entry.getKey())) {
        continue;
      }

      SuperstructureState requestedState = entry.getValue().request().wantedState;
      if (isAimOnlyState(requestedState)
          && (selectedAimSlot == null || entry.getValue().order() > selectedAimSlot.order())) {
        selectedAimOwner = entry.getKey();
        selectedAimSlot = entry.getValue();
      }
      if (requestedState == SuperstructureState.INTAKING
          && (selectedIntakeSlot == null
              || entry.getValue().order() > selectedIntakeSlot.order())) {
        selectedIntakeOwner = entry.getKey();
        selectedIntakeSlot = entry.getValue();
      }
      if (isWristRequest(requestedState)
          && (selectedWristSlot == null || entry.getValue().order() > selectedWristSlot.order())) {
        selectedWristOwner = entry.getKey();
        selectedWristSlot = entry.getValue();
      }

      // Hood/shooter/hopper behavior is the primary lane. Intake and wrist can run beside it.
      if (requestedState == SuperstructureState.INTAKING || isWristRequest(requestedState)) {
        continue;
      }
      if (selectedSlot == null
          || requestPriority(entry.getValue().request()) > requestPriority(selectedSlot.request())
          || (requestPriority(entry.getValue().request()) == requestPriority(selectedSlot.request())
              && entry.getValue().order() > selectedSlot.order())) {
        selectedOwner = entry.getKey();
        selectedSlot = entry.getValue();
      }
    }

    if (selectedSlot == null) {
      if (selectedWristSlot != null
          && (selectedIntakeSlot == null
              || requestPriority(selectedWristSlot.request())
                  >= requestPriority(selectedIntakeSlot.request()))) {
        selectedOwner = selectedWristOwner;
        selectedSlot = selectedWristSlot;
      } else if (selectedIntakeSlot != null) {
        selectedOwner = selectedIntakeOwner;
        selectedSlot = selectedIntakeSlot;
      }
    }

    activeOwner = selectedOwner;
    activeRequest = selectedSlot == null ? SuperstructureRequest.idle() : selectedSlot.request();
    activeGoal = activeRequest.goal;
    auxiliaryAimOwner = selectedAimOwner;
    auxiliaryAimRequest =
        selectedAimSlot == null ? SuperstructureRequest.idle() : selectedAimSlot.request();
    auxiliaryIntakeOwner = selectedIntakeOwner;
    auxiliaryIntakeRequest =
        selectedIntakeSlot == null ? SuperstructureRequest.idle() : selectedIntakeSlot.request();
    auxiliaryWristOwner = selectedWristOwner;
    if (selectedWristSlot != null) {
      auxiliaryWristRequest = selectedWristSlot.request();
    }
  }

  private int requestPriority(SuperstructureRequest request) {
    return switch (request.wantedState) {
      case STATIC_SHOOTING, MANUAL_SHOOTING, PASS_SHOOTING, TUNING_SHOOTING -> 100;
      case WRIST_FORCE_DOWN,
          WRIST_DOWN_NO_STALL,
          WRIST_GOOD_STOW,
          WRIST_MANUAL,
          WRIST_SLOW_UP -> 80;
      case EJECTING -> 70;
      case INTAKING -> 60;
      case STATIC_AIMING, MANUAL_AIMING, PASS_AIMING -> 50;
      case IDLE -> 0;
      default -> 10;
    };
  }

  private void reconcileWantedState() {
    SuperstructureState wanted =
        activeRequest.held ? activeRequest.wantedState : SuperstructureState.IDLE;
    requestState(wanted);

    if (wanted == SuperstructureState.IDLE) {
      if (getCurrentState() != SuperstructureState.IDLE) {
        transition(SuperstructureState.IDLE, SuperstructureTransitionReason.REQUEST_CLEARED);
      }
      return;
    }

    SuperstructureState entryState = convertWantedToEntryState(activeRequest);
    if (!stateMatchesRequest(getCurrentState(), wanted)) {
      transition(entryState, SuperstructureTransitionReason.REQUESTED);
    } else if (isShootingState(getCurrentState()) && isAimOnlyState(wanted)) {
      transition(entryState, SuperstructureTransitionReason.REQUESTED);
    }
  }

  private SuperstructureState convertWantedToEntryState(SuperstructureRequest request) {
    return switch (request.wantedState) {
      case STATIC_SHOOTING -> SuperstructureState.STATIC_AIMING;
      case MANUAL_SHOOTING -> SuperstructureState.MANUAL_AIMING;
      case PASS_SHOOTING -> SuperstructureState.PASS_AIMING;
      case TUNING_SHOOTING -> SuperstructureState.TUNING_AIMING;
      case WRIST_FORCE_DOWN -> SuperstructureState.WRIST_FORCE_DOWN_STEP_1;
      case WRIST_SLOW_UP -> SuperstructureState.WRIST_SLOW_UP_WAIT;
      default -> request.wantedState;
    };
  }

  private boolean stateMatchesRequest(SuperstructureState current, SuperstructureState wanted) {
    return switch (wanted) {
      case STATIC_SHOOTING -> current == SuperstructureState.STATIC_AIMING
          || current == SuperstructureState.STATIC_SHOOTING;
      case MANUAL_SHOOTING -> current == SuperstructureState.MANUAL_AIMING
          || current == SuperstructureState.MANUAL_SHOOTING;
      case PASS_SHOOTING -> current == SuperstructureState.PASS_AIMING
          || current == SuperstructureState.PASS_SHOOTING;
      case TUNING_SHOOTING -> current == SuperstructureState.TUNING_AIMING
          || current == SuperstructureState.TUNING_SHOOTING;
      case WRIST_FORCE_DOWN -> isForceDownStep(current);
      case WRIST_SLOW_UP -> current == SuperstructureState.WRIST_SLOW_UP_WAIT
          || current == SuperstructureState.WRIST_SLOW_UP;
      default -> current == wanted;
    };
  }

  private void runCurrentState() {
    switch (getCurrentState()) {
      case IDLE, FAULTED -> {}
      case INTAKING -> intake.intakeVolts(activeGoal.intakeVolts);
      case EJECTING -> {
        hopper.runHopper(-6.0, 0);
        intake.ejectVolts(6.0);
      }
      case STATIC_AIMING -> runStaticAiming();
      case STATIC_SHOOTING -> runStaticShooting();
      case MANUAL_AIMING -> runManualAiming();
      case MANUAL_SHOOTING -> runManualShooting();
      case PASS_AIMING -> runPassAiming();
      case PASS_SHOOTING -> runPassShooting();
      case TUNING_AIMING -> runTuningAiming();
      case TUNING_SHOOTING -> runTuningShooting();
      case WRIST_MANUAL,
          WRIST_SLOW_UP,
          WRIST_SLOW_UP_WAIT,
          WRIST_DOWN_NO_STALL,
          WRIST_GOOD_STOW,
          WRIST_FORCE_DOWN,
          WRIST_FORCE_DOWN_STEP_1,
          WRIST_FORCE_DOWN_STEP_2,
          WRIST_FORCE_DOWN_STEP_3 -> {
        // Wrist behavior runs in its independent auxiliary lane below.
      }
    }
  }

  private void runAuxiliaryRequests() {
    runAuxiliaryAim();
    runAuxiliaryIntake();
    runAuxiliaryWrist();
  }

  private void runAuxiliaryAim() {
    if (auxiliaryAimOwner != null && auxiliaryAimOwner != activeOwner) {
      switch (auxiliaryAimRequest.wantedState) {
        case STATIC_AIMING -> {
          double distance = hubDistanceMeters();
          hood.setStaticAim(inclineHueristic(distance));
        }
        case MANUAL_AIMING -> hood.setAngle(
            inclineHueristic(auxiliaryAimRequest.goal.manualDistanceMeters));
        case PASS_AIMING -> hood.setPassAim(Rotation2d.fromDegrees(40.0));
        default -> {}
      }
    } else if (previousAuxiliaryAimOwner != null
        && auxiliaryAimOwner == null
        && !stateWritesHood(getCurrentState())) {
      hood.stop();
    }
    previousAuxiliaryAimOwner = auxiliaryAimOwner;
  }

  private void runAuxiliaryIntake() {
    if (auxiliaryIntakeOwner != null && getCurrentState() != SuperstructureState.EJECTING) {
      if (getCurrentState() != SuperstructureState.INTAKING) {
        intake.intakeVolts(auxiliaryIntakeRequest.goal.intakeVolts);
      }
    } else if (previousAuxiliaryIntakeOwner != null
        && getCurrentState() != SuperstructureState.INTAKING
        && getCurrentState() != SuperstructureState.EJECTING) {
      intake.stop();
    }
    previousAuxiliaryIntakeOwner = auxiliaryIntakeOwner;
  }

  private void runAuxiliaryWrist() {
    if (auxiliaryWristOwner == null) {
      if (auxiliaryWristState != SuperstructureState.IDLE) {
        wrist.stop();
      }
      auxiliaryWristRequest = SuperstructureRequest.idle();
      auxiliaryWristState = SuperstructureState.IDLE;
      executingWristOwner = null;
      return;
    }

    SuperstructureState requestedEntry = convertWantedToEntryState(auxiliaryWristRequest);
    if (executingWristOwner != auxiliaryWristOwner) {
      if (executingWristOwner != null) {
        wrist.stop();
      }
      executingWristOwner = auxiliaryWristOwner;
      setAuxiliaryWristState(requestedEntry);
    } else if (!stateMatchesRequest(auxiliaryWristState, auxiliaryWristRequest.wantedState)) {
      setAuxiliaryWristState(requestedEntry);
    }

    switch (auxiliaryWristState) {
      case WRIST_MANUAL -> wrist.runStateVolts(
          WristState.MANUAL, auxiliaryWristRequest.goal.wristManualVolts);
      case WRIST_SLOW_UP_WAIT -> {
        if (auxiliaryWristTimeInState() >= auxiliaryWristRequest.goal.delaySeconds) {
          setAuxiliaryWristState(SuperstructureState.WRIST_SLOW_UP);
        }
      }
      case WRIST_SLOW_UP -> {
        wrist.runStateVolts(WristState.SLOW_UP, 4.0);
        if (inputs.wristAngleDegrees > 90.0) {
          completeAuxiliaryWristRequest(SuperstructureTransitionReason.SENSOR_THRESHOLD);
        }
      }
      case WRIST_DOWN_NO_STALL -> {
        wrist.runStateVolts(WristState.DOWN_NO_STALL, -4.0);
        if (inputs.wristAngleDegrees < 30.0) {
          completeAuxiliaryWristRequest(SuperstructureTransitionReason.SENSOR_THRESHOLD);
        }
      }
      case WRIST_GOOD_STOW -> {
        wrist.runStateVolts(WristState.GOOD_STOW, 5.0);
        if (inputs.wristAngleDegrees > 120.0) {
          completeAuxiliaryWristRequest(SuperstructureTransitionReason.SENSOR_THRESHOLD);
        }
      }
      case WRIST_FORCE_DOWN_STEP_1 -> {
        wrist.runStateVolts(WristState.FORCE_DOWN_STEP_1, -6.0);
        if (auxiliaryWristTimeInState() >= 0.12) {
          setAuxiliaryWristState(SuperstructureState.WRIST_FORCE_DOWN_STEP_2);
        }
      }
      case WRIST_FORCE_DOWN_STEP_2 -> {
        wrist.runStateVolts(WristState.FORCE_DOWN_STEP_2, 8.0);
        if (auxiliaryWristTimeInState() >= 0.12) {
          setAuxiliaryWristState(SuperstructureState.WRIST_FORCE_DOWN_STEP_3);
        }
      }
      case WRIST_FORCE_DOWN_STEP_3 -> {
        wrist.runStateVolts(WristState.FORCE_DOWN_STEP_3, -8.0);
        if (inputs.wristAngleDegrees < 30.0) {
          completeAuxiliaryWristRequest(SuperstructureTransitionReason.SENSOR_THRESHOLD);
        }
      }
      default -> {}
    }

    if (activeOwner == auxiliaryWristOwner
        && auxiliaryWristState != SuperstructureState.IDLE
        && getCurrentState() != auxiliaryWristState) {
      transition(auxiliaryWristState, SuperstructureTransitionReason.INTERNAL_STEP);
    }
  }

  private void setAuxiliaryWristState(SuperstructureState state) {
    auxiliaryWristState = state;
    auxiliaryWristEntryTimestamp = Timer.getFPGATimestamp();
  }

  private double auxiliaryWristTimeInState() {
    if (auxiliaryWristState == SuperstructureState.IDLE) {
      return 0.0;
    }
    return Timer.getFPGATimestamp() - auxiliaryWristEntryTimestamp;
  }

  private synchronized void completeAuxiliaryWristRequest(SuperstructureTransitionReason reason) {
    Object completedOwner = executingWristOwner;
    if (completedOwner != null) {
      completedOwners.add(completedOwner);
    }
    wrist.stop();
    auxiliaryWristOwner = null;
    executingWristOwner = null;
    auxiliaryWristRequest = SuperstructureRequest.idle();
    auxiliaryWristState = SuperstructureState.IDLE;
    if (activeOwner == completedOwner) {
      requestState(SuperstructureState.IDLE);
      transition(SuperstructureState.IDLE, reason);
    }
  }

  private void runStaticAiming() {
    goalDistanceMeters = hubDistanceMeters();
    goalHoodAngleDegrees = inclineHueristic(goalDistanceMeters).getDegrees();
    hood.setStaticAim(Rotation2d.fromDegrees(goalHoodAngleDegrees));
    if (getWantedState() == SuperstructureState.STATIC_SHOOTING) {
      goalRPM = rpmRegress(goalDistanceMeters) + (initialShots ? 200.0 : 0.0);
      shooter.setVelocityRPM(goalRPM);
      transitionToShootingWhenReady(SuperstructureState.STATIC_SHOOTING);
    } else {
      goalRPM = 0.0;
    }
  }

  private void runStaticShooting() {
    goalDistanceMeters = hubDistanceMeters();
    goalHoodAngleDegrees = inclineHueristic(goalDistanceMeters).getDegrees();
    goalRPM = rpmRegress(goalDistanceMeters) + (initialShots ? 200.0 : 0.0);
    hood.setStaticAim(Rotation2d.fromDegrees(goalHoodAngleDegrees));
    shooter.setVelocityRPM(goalRPM);
    shooter.markShooting();
    runFeedLifecycle();
  }

  private void runManualAiming() {
    goalDistanceMeters = activeGoal.manualDistanceMeters;
    goalHoodAngleDegrees = inclineHueristic(goalDistanceMeters).getDegrees();
    hood.setAngle(Rotation2d.fromDegrees(goalHoodAngleDegrees));
    if (getWantedState() == SuperstructureState.MANUAL_SHOOTING) {
      goalRPM = rpmRegress(goalDistanceMeters) + (initialShots ? 300.0 : 0.0);
      shooter.setVelocityRPM(goalRPM);
      transitionToShootingWhenReady(SuperstructureState.MANUAL_SHOOTING);
    } else {
      goalRPM = 0.0;
    }
  }

  private void runManualShooting() {
    goalDistanceMeters = activeGoal.manualDistanceMeters;
    goalHoodAngleDegrees = inclineHueristic(goalDistanceMeters).getDegrees();
    goalRPM = rpmRegress(goalDistanceMeters) + (initialShots ? 300.0 : 0.0);
    hood.setAngle(Rotation2d.fromDegrees(goalHoodAngleDegrees));
    shooter.setVelocityRPM(goalRPM);
    shooter.markShooting();
    runFeedLifecycle();
  }

  private void runPassAiming() {
    goalDistanceMeters = 0.0;
    goalHoodAngleDegrees = 40.0;
    hood.setPassAim(Rotation2d.fromDegrees(goalHoodAngleDegrees));
    if (getWantedState() == SuperstructureState.PASS_SHOOTING) {
      goalRPM = 110.0 * Units.metersToFeet(AllianceFlip.apply(drive.getPose()).getX());
      shooter.setVelocityRPM(goalRPM);
      transitionToShootingWhenReady(SuperstructureState.PASS_SHOOTING);
    } else {
      goalRPM = 0.0;
    }
  }

  private void runPassShooting() {
    goalDistanceMeters = 0.0;
    goalHoodAngleDegrees = 40.0;
    goalRPM = 110.0 * Units.metersToFeet(AllianceFlip.apply(drive.getPose()).getX());
    hood.setPassAim(Rotation2d.fromDegrees(goalHoodAngleDegrees));
    shooter.setVelocityRPM(goalRPM);
    shooter.markShooting();
    runFeedLifecycle();
  }

  private void runTuningAiming() {
    goalDistanceMeters = 0.0;
    goalHoodAngleDegrees = 0.0;
    goalRPM = activeGoal.shooterRPM;
    shooter.setVelocityRPM(goalRPM);
    if (getTimeInState() >= 0.1 && inputs.shooterAtGoal) {
      transition(SuperstructureState.TUNING_SHOOTING, SuperstructureTransitionReason.READY_TO_FEED);
      hopper.runHopper(9.0, 5000);
      shooter.markShooting();
    }
  }

  private void runTuningShooting() {
    goalDistanceMeters = 0.0;
    goalHoodAngleDegrees = 0.0;
    goalRPM = activeGoal.shooterRPM;
    shooter.setVelocityRPM(goalRPM);
    shooter.markShooting();
    hopper.runHopper(9.0, 5000);
  }

  private boolean transitionToShootingWhenReady(SuperstructureState shootingState) {
    if (getTimeInState() >= 0.1 && inputs.shooterAtGoal) {
      transition(shootingState, SuperstructureTransitionReason.READY_TO_FEED);
      runFeedLifecycle();
      shooter.markShooting();
      return true;
    }
    return false;
  }

  private void runFeedLifecycle() {
    hopper.runHopper(9.0, 5000);
    if (getTimeInState() >= 0.1 && inputs.hopperAtGoal) {
      initialShots = false;
    }
  }

  private void transition(SuperstructureState nextState, SuperstructureTransitionReason reason) {
    transitionReason = reason;
    if (!transitionTo(nextState, reason.name())) {
      rejectedWantedState = nextState.name();
      rejectReason = reason.name();
      transitionReason = SuperstructureTransitionReason.REJECTED;
    } else {
      rejectedWantedState = "";
      rejectReason = "";
    }
  }

  @Override
  protected boolean canTransitionTo(SuperstructureState nextState) {
    return getCurrentState() != SuperstructureState.FAULTED
        || nextState == SuperstructureState.FAULTED
        || nextState == SuperstructureState.IDLE;
  }

  @Override
  protected void onExit(SuperstructureState state, SuperstructureState nextState) {
    if (isTuningState(state)) {
      if (!isInternalAimToShootTransition(state, nextState)) {
        shooter.stop();
        hopper.stop();
        initialShots = true;
      }
    } else if (isAimOrShootingState(state)) {
      if (!isInternalAimToShootTransition(state, nextState)) {
        shooter.stop();
        hopper.stop();
        hood.stop();
        initialShots = true;
      }
    } else if (state == SuperstructureState.INTAKING) {
      intake.stop();
    } else if (state == SuperstructureState.EJECTING) {
      hopper.stop();
      intake.stop();
    } else if (isWristState(state) && !(isForceDownStep(state) && isForceDownStep(nextState))) {
      wrist.stop();
    }
  }

  private boolean isInternalAimToShootTransition(
      SuperstructureState state, SuperstructureState nextState) {
    return (state == SuperstructureState.STATIC_AIMING
            && nextState == SuperstructureState.STATIC_SHOOTING)
        || (state == SuperstructureState.MANUAL_AIMING
            && nextState == SuperstructureState.MANUAL_SHOOTING)
        || (state == SuperstructureState.PASS_AIMING
            && nextState == SuperstructureState.PASS_SHOOTING)
        || (state == SuperstructureState.TUNING_AIMING
            && nextState == SuperstructureState.TUNING_SHOOTING);
  }

  private boolean isAimOrShootingState(SuperstructureState state) {
    return switch (state) {
      case STATIC_AIMING,
          STATIC_SHOOTING,
          MANUAL_AIMING,
          MANUAL_SHOOTING,
          PASS_AIMING,
          PASS_SHOOTING,
          TUNING_AIMING,
          TUNING_SHOOTING -> true;
      default -> false;
    };
  }

  private boolean isTuningState(SuperstructureState state) {
    return state == SuperstructureState.TUNING_AIMING
        || state == SuperstructureState.TUNING_SHOOTING;
  }

  private boolean stateWritesHood(SuperstructureState state) {
    return switch (state) {
      case STATIC_AIMING,
          STATIC_SHOOTING,
          MANUAL_AIMING,
          MANUAL_SHOOTING,
          PASS_AIMING,
          PASS_SHOOTING -> true;
      default -> false;
    };
  }

  private boolean isShootingState(SuperstructureState state) {
    return state == SuperstructureState.STATIC_SHOOTING
        || state == SuperstructureState.MANUAL_SHOOTING
        || state == SuperstructureState.PASS_SHOOTING
        || state == SuperstructureState.TUNING_SHOOTING;
  }

  private boolean isAimOnlyState(SuperstructureState state) {
    return state == SuperstructureState.STATIC_AIMING
        || state == SuperstructureState.MANUAL_AIMING
        || state == SuperstructureState.PASS_AIMING;
  }

  private boolean isWristState(SuperstructureState state) {
    return state == SuperstructureState.WRIST_MANUAL
        || state == SuperstructureState.WRIST_SLOW_UP
        || state == SuperstructureState.WRIST_SLOW_UP_WAIT
        || state == SuperstructureState.WRIST_DOWN_NO_STALL
        || state == SuperstructureState.WRIST_GOOD_STOW
        || state == SuperstructureState.WRIST_FORCE_DOWN_STEP_1
        || state == SuperstructureState.WRIST_FORCE_DOWN_STEP_2
        || state == SuperstructureState.WRIST_FORCE_DOWN_STEP_3;
  }

  private boolean isWristRequest(SuperstructureState state) {
    return state == SuperstructureState.WRIST_MANUAL
        || state == SuperstructureState.WRIST_SLOW_UP
        || state == SuperstructureState.WRIST_DOWN_NO_STALL
        || state == SuperstructureState.WRIST_GOOD_STOW
        || state == SuperstructureState.WRIST_FORCE_DOWN;
  }

  private boolean isForceDownStep(SuperstructureState state) {
    return state == SuperstructureState.WRIST_FORCE_DOWN_STEP_1
        || state == SuperstructureState.WRIST_FORCE_DOWN_STEP_2
        || state == SuperstructureState.WRIST_FORCE_DOWN_STEP_3;
  }

  private double hubDistanceMeters() {
    return AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
        .minus(drive.getPose().getTranslation())
        .getNorm();
  }

  public static double rpmRegress(double distanceMeters) {
    return 38.0 * Math.pow(distanceMeters - 1.5, 2) + 1800.0;
  }

  public static Rotation2d inclineHueristic(double distanceMeters) {
    return Rotation2d.fromRadians(Math.PI / 2.0 - Math.atan(7.0 / distanceMeters));
  }

  private void stopAll() {
    shooter.stop();
    hopper.stop();
    intake.stop();
    hood.stop();
    wrist.stop();
    initialShots = true;
  }

  private SuperstructureMode getMode() {
    if (DriverStation.isDisabled()) return SuperstructureMode.DISABLED;
    if (DriverStation.isAutonomous()) return SuperstructureMode.AUTONOMOUS;
    if (DriverStation.isTest()) return SuperstructureMode.TEST;
    return SuperstructureMode.TELEOP;
  }

  @Override
  protected void logAdditionalOutputs() {
    Logger.recordOutput("Superstructure/Mode", getMode().name());
    Logger.recordOutput("Superstructure/TransitionReason", transitionReason.name());
    Logger.recordOutput("Superstructure/RejectedWantedState", rejectedWantedState);
    Logger.recordOutput("Superstructure/RejectReason", rejectReason);
    Logger.recordOutput("Superstructure/InitialShots", initialShots);
    Logger.recordOutput("Superstructure/ShooterAtGoal", inputs.shooterAtGoal);
    Logger.recordOutput("Superstructure/HopperAtGoal", inputs.hopperAtGoal);
    Logger.recordOutput("Superstructure/WristAngle", inputs.wristAngleDegrees);
    Logger.recordOutput("Superstructure/AuxiliaryWristState", auxiliaryWristState.name());
    Logger.recordOutput("Superstructure/AuxiliaryWristTimeInState", auxiliaryWristTimeInState());
    Logger.recordOutput("Superstructure/AuxiliaryIntakeActive", auxiliaryIntakeOwner != null);
    Logger.recordOutput("Superstructure/AuxiliaryAimActive", auxiliaryAimOwner != null);
    Logger.recordOutput(
        "Superstructure/AuxiliaryIntakeVolts", auxiliaryIntakeRequest.goal.intakeVolts);
    Logger.recordOutput("Superstructure/GoalDistanceMeters", goalDistanceMeters);
    Logger.recordOutput("Superstructure/GoalRPM", goalRPM);
    Logger.recordOutput("Superstructure/GoalHoodAngleDeg", goalHoodAngleDegrees);
  }
}
