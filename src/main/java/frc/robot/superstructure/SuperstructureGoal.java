package frc.robot.superstructure;

/** Numeric data associated with a superstructure request. */
public final class SuperstructureGoal {
  public final double manualDistanceMeters;
  public final double wristManualVolts;
  public final double intakeVolts;
  public final double shooterRPM;
  public final double delaySeconds;
  public final boolean useManualDistance;
  public final boolean fromAuto;

  public SuperstructureGoal(
      double manualDistanceMeters,
      double wristManualVolts,
      double intakeVolts,
      double shooterRPM,
      double delaySeconds,
      boolean useManualDistance,
      boolean fromAuto) {
    this.manualDistanceMeters = manualDistanceMeters;
    this.wristManualVolts = wristManualVolts;
    this.intakeVolts = intakeVolts;
    this.shooterRPM = shooterRPM;
    this.delaySeconds = delaySeconds;
    this.useManualDistance = useManualDistance;
    this.fromAuto = fromAuto;
  }

  public static SuperstructureGoal none() {
    return new SuperstructureGoal(0.0, 0.0, 0.0, 0.0, 0.0, false, false);
  }

  public SuperstructureGoal asAuto() {
    return new SuperstructureGoal(
        manualDistanceMeters,
        wristManualVolts,
        intakeVolts,
        shooterRPM,
        delaySeconds,
        useManualDistance,
        true);
  }
}
