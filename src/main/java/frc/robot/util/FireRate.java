package frc.robot.util;

import frc.robot.Constants;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FireRate {
  private final double singleShot = Constants.ShooterConstants.singleShotAcceleration;

  private final double doubleShot = Constants.ShooterConstants.doubleShotAcceleration;

  private final Shooter shooter;

  private int fuelShot = 0;
  private int cyclesSinceLastCount = 0;

  private double cyclePeak = 0.0;
  private double peakRPM = 0.0;

  private final List<Integer> previousShots = new ArrayList<>();

  public FireRate(Shooter shooter) {
    this.shooter = shooter;
  }

  public boolean isFiring() {
    if (previousShots.size() < 2) {
      return false;
    }

    int oldest = previousShots.get(0);
    int newest = previousShots.get(previousShots.size() - 1);

    return newest > oldest;
  }

  public void periodic() {
    double acceleration = shooter.getAverageAcceleration();

    if (acceleration > cyclePeak) {
      cyclePeak = acceleration;
      peakRPM = shooter.getSpeeds();
    }

    cyclesSinceLastCount++;

    if (cyclesSinceLastCount >= 10) {
      ShotType shotType = shotDesignator(cyclePeak, peakRPM);

      switch (shotType) {
        case ONE:
          fuelShot++;
          break;

        case TWO:
          fuelShot += 2;
          break;

        case NONE:
        default:
          break;
      }

      if (previousShots.size() >= 10) {
        previousShots.remove(0);
      }

      previousShots.add(fuelShot);

      cyclesSinceLastCount = 0;
      cyclePeak = 0.0;
      peakRPM = 0.0;
    }

    Logger.recordOutput("Shooter/isFiring?", isFiring());
    Logger.recordOutput("Shooter/FuelCount", fuelShot);
    Logger.recordOutput("Shooter/FireRate", getRate());
    Logger.recordOutput("Shooter/CyclePeakAcceleration", cyclePeak);
    Logger.recordOutput("Shooter/PeakRPM", peakRPM);
  }

  public int getFuelCount() {
    return fuelShot;
  }

  public double getRate() {
    if (previousShots.size() < 2) {
      return 0.0;
    }

    int oldest = previousShots.get(0);
    int newest = previousShots.get(previousShots.size() - 1);

    double shots = newest - oldest;

    // change this to the actual time in the shot window
    double windowSeconds = 2.0;

    return shots / windowSeconds;
  }

  public enum ShotType {
    NONE,
    ONE,
    TWO
  }

  public ShotType shotDesignator(double acceleration, double rpm) {
    double targetRPM = ScoringCommands.getRegressionRPM();

    boolean isAtSpeed = targetRPM > 0 && Math.abs(rpm - targetRPM) <= 0.10 * targetRPM;

    if (!isAtSpeed) {
      return ShotType.NONE;
    }

    if (acceleration >= doubleShot) {
      return ShotType.TWO;
    }

    if (acceleration >= singleShot) {
      return ShotType.ONE;
    }

    return ShotType.NONE;
  }
}
