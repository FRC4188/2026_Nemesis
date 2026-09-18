package frc.robot.commands.Scoring;

import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  // please ignore this bs
  private double nominal = frc.robot.Constants.ShooterConstants.idleAcceleration;
  private double singleShot = frc.robot.Constants.ShooterConstants.singleShotAcceleration;
  private double doubleShot = frc.robot.Constants.ShooterConstants.doubleShotAcceleration;

  private Shooter shooter;

  private int fuelShot = 0;

  private int cyclesSinceLastCount = 0;

  private double cyclePeak = 0.0;

  private List<Integer> previousShots = new ArrayList<>();

  public ShotCalculator(Shooter shooter) {
    this.shooter = shooter;
  }

  public boolean isFiring() {
    //5 cycles = 1 second. this is buffer
    if (previousShots.size() < 5) {
      return true;
    }
    return previousShots.get(previousShots.size() - 1) > previousShots.get(0);
  }

  public void periodic() {
    if (shooter.getAverageAcceleration() > cyclePeak) {
      cyclePeak = shooter.getAverageAcceleration();
    }

    shotType shotType = shotDesignator(cyclePeak);
    cyclesSinceLastCount++;

    if (cyclesSinceLastCount > 10) {
      switch (shotType) {
        case ONE:
          fuelShot++;
          break;
        case TWO:
          fuelShot += 2;
          break;
        default:
          break;
      }
      // around 2 seconds of indexing, i can add more.
      if (previousShots.size() >= 10) {
        previousShots.remove(0);
      }
      previousShots.add(fuelShot);
      cyclesSinceLastCount = 0;
      cyclePeak = 0;
    }
    Logger.recordOutput("Shooter/isFiring?", isFiring());
    Logger.recordOutput("Shooter/FuelCount", fuelShot);
  }

  public int getFuelCount() {
    return fuelShot;
  }

  public enum shotType {
    NONE,
    ONE,
    TWO
  }
  // TODO: WIP
  public shotType shotDesignator(double acceleration) {
    double targetRPM = ScoringCommands.getRegressionRPM();
    double currentRPM = shooter.getSpeeds();
    // margin of error is 10% rn priyanshu pls tell me the acc number
    boolean isAtSpeed = targetRPM > 0 && Math.abs(currentRPM - targetRPM) <= (0.10 * targetRPM);

    if (isAtSpeed) {

      if (acceleration >= singleShot && acceleration < doubleShot) {
        return shotType.ONE;
      } else if (acceleration >= doubleShot) {
        return shotType.TWO;
      }
    }

    return shotType.NONE;
  }
}
