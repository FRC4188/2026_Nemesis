package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.statemachine.StateMachine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  public enum State {
    IDLE,
    HOLDING,
    POSITION,
    STOWING,
    MANUAL,
    COAST_VOLTAGE,
    LOWER_COAST,
    LOWER_BANG,
    FORCE_LOWER,
    COMPRESS,
    COAST,
    ZEROING
  }

  private static Wrist instance = null;

  public static synchronized Wrist getInstance() {
    if (instance == null)
      instance =
          new Wrist(
              switch (Constants.Robot.currentMode) {
                case REAL -> new WristIOReal();
                case SIM -> new WristIOSim();
                case REPLAY -> new WristIO() {};
              });
    return instance;
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs;
  private final Alert wristDisconnectedAlert;

  // @AutoLogOutput(key = "Wrist/Setpoint Degrees")
  private double setpoint = 0.0;
  private double wantedVolts;
  private double wantedPosition;
  private boolean compressionPaused;
  private double compressionReverseStart;
  private double compressionReverseDuration;
  private double compressionReverseVolts;
  private double compressionActiveStart;
  private boolean lowerIntakeComplete = true;
  private final StateMachine<State> stateMachine =
      new StateMachine<>("Wrist", State.IDLE) {
        @Override
        protected void onEnter(State state, State previousState) {
          if (state == State.COMPRESS) {
            compressionReverseStart = 0.0;
            compressionActiveStart = Timer.getFPGATimestamp();
          }
        }

        @Override
        protected void runStateMachine() {
          if (getCurrentState() != getWantedState()) {
            transitionTo(getWantedState(), "Requested");
          }

          double timeInState = getTimeInState();
          switch (getCurrentState()) {
            case IDLE -> io.setVolts(0.0);
            case HOLDING -> io.setVolts(-0.3);
            case POSITION -> io.setPosition(Rotation2d.fromDegrees(wantedPosition));
            case STOWING -> {
              if (getAngle() > 120.0) {
                requestState(State.HOLDING);
              } else {
                io.setVolts(5.0);
              }
            }
            case MANUAL -> io.setVolts(wantedVolts);
            case COAST_VOLTAGE -> io.setCoastVolts(wantedVolts);
            case LOWER_COAST, LOWER_BANG -> {
              boolean bang = getCurrentState() == State.LOWER_BANG;
              if (timeInState < 0.15) {
                io.setVolts(bang ? -10.0 : -8.0);
              } else {
                io.setCoastVolts(bang ? -4.0 : -3.5);
                if (Math.abs(getVelocityDegPerSec()) < 0.007) {
                  io.setZero();
                  lowerIntakeComplete = true;
                  requestState(State.IDLE);
                }
              }
            }
            case FORCE_LOWER -> {
              double volts = timeInState < 0.12 ? -8.0 : timeInState < 0.24 ? 8.0 : -8.0;
              if (getAngle() < 30.0
                  || getStatorCurrent() > Constants.WristConstants.bumperStatorCurrent) {
                io.setVolts(0.0);
                requestState(State.IDLE);
              } else {
                io.setVolts(volts);
              }
            }
            case COMPRESS -> runCompression();
            case COAST -> {
              io.setCoast();
              requestState(State.IDLE);
            }
            case ZEROING -> {
              io.setZero();
              requestState(State.IDLE);
            }
          }
        }
      };

  @AutoLogOutput(key = "Wrist/Shake Enable?")
  public boolean shakeEnable = true;

  public void enableShake(boolean enable) {
    shakeEnable = enable;
  }

  public Wrist(WristIO io) {
    this.io = io;
    inputs = new WristIOInputsAutoLogged();
    wristDisconnectedAlert = new Alert("Wrist motor disconnected.", AlertType.kError);
  }

  public void runWristVolts(double volts) {
    setpoint = 180;
    wantedVolts = volts;
    stateMachine.requestState(State.MANUAL);
  }

  public void stop() {
    stateMachine.requestState(State.IDLE);
  }

  public void stow() {
    setpoint = 144;
    stateMachine.requestState(State.STOWING);
  }

  public void down() {
    setpoint = 0;
    wantedPosition = Constants.WristConstants.Min_A.getDegrees();
    stateMachine.requestState(State.POSITION);
  }

  public void setAngle(double set) {
    setpoint = set;
    wantedPosition = set;
    stateMachine.requestState(State.POSITION);
  }

  public void setCoast() {
    stateMachine.requestState(State.COAST);
  }

  public void setCoastVolts(double volts) {
    wantedVolts = volts;
    stateMachine.requestState(State.COAST_VOLTAGE);
  }

  public void requestHold() {
    stateMachine.requestState(State.HOLDING);
  }

  public void requestLowerIntake(boolean bang) {
    lowerIntakeComplete = false;
    stateMachine.requestState(bang ? State.LOWER_BANG : State.LOWER_COAST);
  }

  public boolean isLowerIntakeComplete() {
    return lowerIntakeComplete;
  }

  public void requestForceLower() {
    stateMachine.requestState(State.FORCE_LOWER);
  }

  public void requestCompression(boolean paused) {
    if (paused != compressionPaused && !paused) {
      compressionActiveStart = Timer.getFPGATimestamp();
    }
    compressionPaused = paused;
    stateMachine.requestState(State.COMPRESS);
  }

  public boolean isCompressionReady() {
    return stateMachine.getCurrentState() == State.COMPRESS
        && !compressionPaused
        && Timer.getFPGATimestamp() - compressionActiveStart >= 1.5;
  }

  private void runCompression() {
    if (compressionPaused || Timer.getFPGATimestamp() - compressionActiveStart < 1.5) {
      io.setVolts(0.0);
      return;
    }

    if (compressionReverseStart > 0.0) {
      if (Timer.getFPGATimestamp() - compressionReverseStart < compressionReverseDuration) {
        io.setVolts(compressionReverseVolts);
        return;
      }
      compressionReverseStart = 0.0;
    }

    if (getAngle() >= 110.0) {
      io.setVolts(0.0);
    } else if (getStatorCurrent() > Constants.WristConstants.fuelStatorCurrent) {
      compressionReverseStart = Timer.getFPGATimestamp();
      compressionReverseDuration = getAngle() > 90.0 ? 0.6 : Double.POSITIVE_INFINITY;
      compressionReverseVolts = getAngle() > 90.0 ? -2.0 : -0.5;
      io.setVolts(compressionReverseVolts);
    } else {
      io.setVolts(1.15);
    }
  }

  @AutoLogOutput(key = "Wrist/Velocity")
  public double getVelocityDegPerSec() {
    return inputs.motorVelocity.in(Units.DegreesPerSecond);
  }

  public void zero() {
    stateMachine.requestState(State.ZEROING);
  }

  public double getStatorCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput(key = "Wrist/Angle Degrees")
  public double getAngle() {
    return inputs.position.getDegrees();
  }

  // @AutoLogOutput(key = "Wrist/At Goal?")
  public boolean atGoal() {
    return Math.abs(inputs.position.getDegrees() - setpoint)
        < Constants.WristConstants.kTolerance.getDegrees();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    stateMachine.periodic();

    wristDisconnectedAlert.set(!inputs.connected);
  }

  public State getWantedState() {
    return stateMachine.getWantedState();
  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }
}
