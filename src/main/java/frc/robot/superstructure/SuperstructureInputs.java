package frc.robot.superstructure;

import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

/** Sensor-derived values sampled at the start of each Superstructure loop. */
public final class SuperstructureInputs {
  public boolean shooterAtGoal;
  public boolean hopperAtGoal;
  public double wristAngleDegrees;
  public boolean faulted;

  public void update(Shooter shooter, Hopper hopper, Wrist wrist) {
    shooterAtGoal = shooter.atGoal();
    hopperAtGoal = hopper.indexAtGoal();
    wristAngleDegrees = wrist.getAngle();
    faulted = false;
  }
}
