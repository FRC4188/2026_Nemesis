package frc.robot.util;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class BPSCalculator {
    private final Shooter shooter;
    private boolean isShooting = false; //yes i took this from el bean

    private int fuelInInterval = 0;
    private Timer timer = new Timer();
    private double startTime = timer.get();
    private double currentBPS = 0;

    private int totalFuelShot = 0;

    public BPSCalculator(Shooter shooter) {
        this.shooter = shooter;
    }

    private void countFuel() {
        //acceleration over velocity because velocity varies (way more than accel at least)
        double acceleration = shooter.getAcceleration();

        if(!isShooting && acceleration < Constants.ShooterConstants.kFuelAccelerationDipThreshold) {
            isShooting = true;
            fuelInInterval++;
            totalFuelShot++;
        } else if(Math.abs(acceleration) < Constants.ShooterConstants.kAccelerationTolerance) {
            isShooting = false;
        }
    }

    private void update() {        
        double now = timer.get();
        double elapsedTime = now - startTime;

        if(elapsedTime >= 0.4) {
            currentBPS = fuelInInterval / elapsedTime;
            fuelInInterval = 0;
            startTime = now;
        }
    }

    public void periodic() {
        countFuel();
        update();
    }

    @AutoLogOutput(key = "Shooter/BPS")
    public double getCurrentBPS() {
        return currentBPS;
    }

    @AutoLogOutput(key = "Shooter/Total Shot")
    public int getTotalShot() {
        return totalFuelShot;
    }
}