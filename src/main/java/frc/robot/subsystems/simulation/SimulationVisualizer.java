package frc.robot.subsystems.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SimulationVisualizer {
  private String key;

  private DoubleSupplier wristpos;
  private DoubleSupplier hoodpos;
  private DoubleSupplier climberpos;

  public SimulationVisualizer(
      String logkey, DoubleSupplier wrist, DoubleSupplier hood, DoubleSupplier climber) {
    wristpos = wrist;
    hoodpos = hood;
    climberpos = climber;

    key = logkey;
  }

  public void update() {

    Pose3d wristPos =
        SimulationConfig.wristAxis.transformBy(
            new Transform3d(
                new Translation3d(0, 0, 0), new Rotation3d(0, wristpos.getAsDouble(), 0)));

    Pose3d hoodPos =
        SimulationConfig.hoodAxis.transformBy(
            new Transform3d(
                new Translation3d(0, 0, 0), new Rotation3d(0, hoodpos.getAsDouble(), 0)));

    Pose3d climberPos =
        SimulationConfig.climberAxis.transformBy(
            new Transform3d(new Translation3d(), new Rotation3d(0, climberpos.getAsDouble(), 0)));

    Pose3d hopperPos =
        SimulationConfig.hopper.transformBy(
            new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));

    // might need to add agitation, and indexer
    Logger.recordOutput("Mechanism3d/" + key, wristPos, hoodPos, climberPos, hopperPos);
  }
}
