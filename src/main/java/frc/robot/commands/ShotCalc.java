import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class ShotCalc {
    private static final Shooter shooter = Shooter.getInstance();
    private static final Hopper hopper = Hopper.getInstance();
    private static final Drive drive = Drive.getInstance();
    private static final Hood hood = Hood.getInstance();
    private static final Wrist wrist = Wrist.getInstance();
    private static final Intake intake = Intake.getInstance();


    public static final double kMPSAt5000RPM = 0.0; // Placeholder value, replace with actual value

    public static final double kApexHeight = Units.inchesToMeters(100); // 100-120 inches is best range (decide after testing MPS)

    public static double factorEstimatedDrag(double velocity) {
        return velocity / (1 + (0.0023769 * 0.47 * 0.0295) / (2 * 0.0147));
    }

    

    
}