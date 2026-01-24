package frc.robot.subsystems.Loader;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Loader.Intake.Intake;
import frc.robot.subsystems.Loader.Intake.IntakeIO;
import frc.robot.subsystems.Loader.Wrist.Wrist;
import frc.robot.subsystems.Loader.Wrist.WristIO;

public class Loader extends SubsystemBase {
  private final Intake intake;
  private final Wrist wrist;

  public Loader(IntakeIO intakeIO, WristIO wristIO) {
    intake = new Intake(intakeIO);
    wrist = new Wrist(wristIO);
  }

  @Override
  public void periodic() {
    intake.periodic();
    wrist.periodic();
  }

  public void runWrist(double volts) {
    wrist.runVolts(volts);
  }

  public void runIntake(double volts) {
    intake.runVolts(volts);
  }

  public void setWrist(Rotation2d radians) {
    wrist.setPosition(radians);
  }

  public boolean atWristGoal(Rotation2d radians, double tolerance) {
    return Math.abs((wrist.getAngle() - radians.getRadians())) < tolerance;
  }

  public boolean atWristGoal(Rotation2d radians) {
    return Math.abs(wrist.getAngle() - radians.getRadians()) < Constants.WristConstants.kTolerance;
  }

  public void updateWristPID(double kp, double ki, double kd, double kg) {
    wrist.updatePID(kp, ki, kd, kg);
  }
}
