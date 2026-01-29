package frc.robot.subsystems.Transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Transfer.Hopper.Hopper;
import frc.robot.subsystems.Transfer.Hopper.HopperIO;
import frc.robot.subsystems.Transfer.Indexer.Indexer;
import frc.robot.subsystems.Transfer.Indexer.IndexerIO;

public class Transfer extends SubsystemBase {
  private final Hopper hopper;
  private final Indexer indexer;

  public Transfer(HopperIO hopperIO, IndexerIO indexerIO) {
    hopper = new Hopper(hopperIO);
    indexer = new Indexer(indexerIO);
  }

  @Override
  public void periodic() {
    hopper.periodic();
    indexer.periodic();
  }

  public void runIndexer(double volts) {
    indexer.runVolts(volts);
  }

  public void aggitate(double volts) {
    hopper.runVolts(volts);
  }

  public void stopIndexer() {
    indexer.stop();
  }

  public void stopAgitation() {
    hopper.stop();
  }
}
