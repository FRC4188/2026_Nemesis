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
        new Pose3d(
            SimulationConfig.wristAxis.getTranslation(),
            new Rotation3d(0, -wristpos.getAsDouble(), 0));

    Pose3d hopperPos = SimulationConfig.hopper;

    Pose3d hoodPos =
        new Pose3d(
            SimulationConfig.hoodAxis.getTranslation(),
            new Rotation3d(0, hoodpos.getAsDouble()-0.35, 0));

    Pose3d climberPos =
        SimulationConfig.climberAxis.transformBy(
            new Transform3d(new Translation3d(0, climberpos.getAsDouble(), 0), Rotation3d.kZero));

    Pose3d agitatorPos = SimulationConfig.agitator;

    Pose3d indexerPos = SimulationConfig.indexer;

    Logger.recordOutput(
        "Mechanism3d/" + key, wristPos, hopperPos, hoodPos, climberPos, agitatorPos, indexerPos);
  }
}
