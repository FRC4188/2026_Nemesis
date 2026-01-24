package frc.robot.subsystems.Transfer.Indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;
  private final Timer timer;

  public Indexer(IndexerIO io) {
    this.io = io;
    inputs = new IndexerIOInputsAutoLogged();
    timer = new Timer();
  }

  public void runVolts(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    io.runVolts(volts);
  }

  public void stop() {
    io.runVolts(0);
  }

  @AutoLogOutput(key = "Indexer/Is Stalled?")
  public boolean isStalled() {
    return io.isStalled();
  }

  public void periodic() {
    io.UpdateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }
}
