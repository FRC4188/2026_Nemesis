package frc.robot.CSPLib.pidtuning;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PIDTuning {
  private DoubleSupplier currentValue;
  private PIDValueConsumer updateValue;
  private DoubleConsumer setValue;

  private LoggedNetworkNumber loggedkP;
  private LoggedNetworkNumber loggedkI;
  private LoggedNetworkNumber loggedkD;
  private LoggedNetworkNumber loggedkF;
  private LoggedNetworkNumber loggedkTarget;

  private double kP = 1.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kF = 0.0;
  private double kTarget = 0.0;

  private String system = "";

  public PIDTuning() {}

  public PIDTuning(
      String systemName, DoubleSupplier current, DoubleConsumer set, PIDValueConsumer update) {
    currentValue = current;
    setValue = set;
    updateValue = update;
    system = systemName;

    loggedkP = new LoggedNetworkNumber("Tuning " + system + "/kP", kP);
    loggedkI = new LoggedNetworkNumber("Tuning " + system + "/kI", kI);
    loggedkD = new LoggedNetworkNumber("Tuning " + system + "/kD", kD);
    loggedkF = new LoggedNetworkNumber("Tuning " + system + "/(Velocity: kV, Position: kG)", kF);
    loggedkTarget = new LoggedNetworkNumber("Tuning " + system + "/Target Value", kTarget);
  }

  public void updateLoop() {
    Logger.recordOutput("Tuning " + system + "/Current Value", currentValue.getAsDouble());

    if (kP != loggedkP.get()
        || kI != loggedkI.get()
        || kD != loggedkD.get()
        || kF != loggedkF.get()
        || kTarget != loggedkTarget.get()) {

      kP = loggedkP.get();
      kI = loggedkI.get();
      kD = loggedkD.get();
      kF = loggedkF.get();
      kTarget = loggedkTarget.get();
      updateValue.accept(kP, kI, kD, kF);
      setValue.accept(kTarget);
    }
  }

  @FunctionalInterface
  public static interface PIDValueConsumer {
    public void accept(double kP, double kI, double kD, double kF);
  }
}
