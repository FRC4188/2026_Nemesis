package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Loader.Loader;
import frc.robot.subsystems.Transfer.Transfer;

public class LoadingCommands {

    
    public Command pivot(Loader loader, Rotation2d angle){
        return Commands.runOnce(() -> loader.setWrist(angle), loader);
    }
    public Command load(Loader loader, double volt){
        return Commands.runOnce(() -> loader.intake(volt), loader);
    }
    public Command index(Transfer transfer, double volt){
        return Commands.runOnce(() -> transfer.aggitate(volt), transfer);
    }
    
    
}
