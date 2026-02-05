package frc.robot.commands.Scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.Launcher.Hood.Hood;
import frc.robot.subsystems.Loader.Loader;

public class WindupCommand extends SequentialCommandGroup{

    public WindupCommand (Launcher launcher){
        addRequirements();

        



    }

    private class HoodState extends Command{
        private Launcher launcher;
        private double angle;

        public HoodState(Launcher launcher, double angle){
            this.launcher = launcher;
            this.angle = MathUtil.clamp(angle, HoodConstants.Min_A, HoodConstants.Max_A);
        }

        public void initialize(){
            launcher.setHoodAngle(angle);
            
        }
        public boolean isFinished(){
            return launcher.hoodAtTarget();
        }
    }

    // private static class WristState extends Command {
    //     private Loader loader;
    //     private double angle;

    //     public void initialize(){
    //         loader.setAngle(angle);
    //     }

    //     public boolean isFinished(){
    //         return loader.wristAtTarget();
    //     }
    // }

    
}
