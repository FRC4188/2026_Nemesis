package frc.robot.commands.Scoring;

import frc.robot.subsystems.shooter.Shooter;

public class ShotCalculator {
    //please ignore this bs
    private double nominal = frc.robot.Constants.ShooterConstants.idleAcceleration;
    private double singleShot = frc.robot.Constants.ShooterConstants.singleShotAcceleration;
    private double doubleShot = frc.robot.Constants.ShooterConstants.doubleShotAcceleration;
    
    private Shooter shooter = Shooter.getInstance(); 

    private int fuelShot = 0;

    private int cyclesSinceLastCount = 0;

    private double cyclePeak = 0.0;
    
    public ShotCalculator(){
        
    }
    public void periodic(){
        if(shooter.getAverageAcceleration()>cyclePeak){
            cyclePeak = shooter.getAverageAcceleration();
        }

        shotType shotType = shotDesignator(cyclePeak, shooter.getSpeeds());
        cyclesSinceLastCount++;
        
        if(cyclesSinceLastCount>10){
            switch (shotType) {
                case ONE:
                    fuelShot++;
                    break;
                case TWO:
                    fuelShot+=2;
                    break;
                default:
                    break;
            }
            cyclesSinceLastCount = 0;
            cyclePeak = 0;
        }
    }

    public int getFuelCount(){
        return fuelShot;
    }
    public enum shotType {  
        NONE,
        ONE,
        TWO
    }
    
    public shotType shotDesignator(double acceleration, double flywheelspeed){
        if(flywheelspeed>shooter.getSpeeds()){
            if(nominal<singleShot && acceleration<doubleShot){
                return shotType.ONE;
            }else if(doubleShot<acceleration){
                return shotType.TWO;
            }else{
                return shotType.NONE;
        }
        }
        return shotType.NONE;
        
    }
}
