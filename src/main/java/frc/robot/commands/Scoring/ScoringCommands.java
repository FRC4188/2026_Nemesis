package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureRequest;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ScoringCommands {
  private static final Shooter shooter = Shooter.getInstance();
  private static final Hopper hopper = Hopper.getInstance();
  private static final Hood hood = Hood.getInstance();
  private static final Wrist wrist = Wrist.getInstance();
  private static final Intake intake = Intake.getInstance();
  private static final Superstructure superstructure = Superstructure.getInstance();

  public static LoggedNetworkNumber _RPM = new LoggedNetworkNumber("Aim Tuning/RPM", 0.0);

  public static Command dataShoot() {
    return heldRequest(() -> SuperstructureRequest.dataShoot(_RPM.getAsDouble()), shooter, hopper);
  }

  public static Command staticAim() {
    return heldRequest(SuperstructureRequest::staticAim, hood);
  }

  public static boolean initialShots = true;

  public static Command staticShoot() {
    return heldRequest(SuperstructureRequest::staticShoot, shooter, hopper);
  }

  public static Command manualAim(DoubleSupplier distance) {
    return heldRequest(
        () -> SuperstructureRequest.manualAim(Units.feetToMeters(distance.getAsDouble())), hood);
  }

  public static Command manualShoot(DoubleSupplier distance) {
    return heldRequest(
        () -> SuperstructureRequest.manualShoot(Units.feetToMeters(distance.getAsDouble())),
        shooter,
        hopper);
  }

  public static double RPMRegress(double distance) {
    return Superstructure.rpmRegress(distance);
  }

  public static Rotation2d inclineHueristic(double distance) {
    return Superstructure.inclineHueristic(distance);
  }

  public static Command passAim() {
    return heldRequest(SuperstructureRequest::passAim, hood);
  }

  public static Command passShoot() {
    return heldRequest(SuperstructureRequest::passShoot, shooter, hopper);
  }

  public static Command intake() {
    return heldRequest(SuperstructureRequest::intake, intake);
  }

  public static Command intakeVolts(double volts) {
    return heldRequest(() -> SuperstructureRequest.intake(volts), intake);
  }

  static Command intakeVoltsWithoutRequirement(double volts) {
    return heldRequest(() -> SuperstructureRequest.intake(volts));
  }

  public static Command eject() {
    return heldRequest(SuperstructureRequest::eject, hopper, intake);
  }

  public static Command wristManual(DoubleSupplier volts) {
    return heldRequest(() -> SuperstructureRequest.wristManual(volts.getAsDouble()), wrist);
  }

  public static Command slowUp(AutoCommands.Size size) {
    double delaySeconds =
        switch (size) {
          case PRE -> 0.5;
          case HALF -> 1.5;
          case FULL -> 4.0;
        };
    return Commands.either(
        finiteRequest(() -> SuperstructureRequest.wristSlowUp(delaySeconds), wrist)
            .alongWith(intakeVolts(5.0)),
        Commands.none(),
        () -> wrist.shakeEnable);
  }

  public static Command downNoStall() {
    return finiteRequest(SuperstructureRequest::wristDownNoStall, wrist);
  }

  public static Command forceDown() {
    return finiteRequest(SuperstructureRequest::wristForceDown, wrist);
  }

  public static Command goodStow() {
    return finiteRequest(SuperstructureRequest::wristGoodStow, wrist);
  }

  private static Command heldRequest(
      Supplier<SuperstructureRequest> requestSupplier, Subsystem... requirements) {
    Object owner = new Object();
    return Commands.runEnd(
        () -> superstructure.request(owner, requestForCurrentMode(requestSupplier)),
        () -> superstructure.clearRequest(owner),
        requirements);
  }

  private static Command finiteRequest(
      Supplier<SuperstructureRequest> requestSupplier, Subsystem... requirements) {
    Object owner = new Object();
    return new FunctionalCommand(
        () -> superstructure.request(owner, requestForCurrentMode(requestSupplier)),
        () -> superstructure.request(owner, requestForCurrentMode(requestSupplier)),
        interrupted -> superstructure.clearRequest(owner),
        () -> superstructure.isRequestComplete(owner),
        requirements);
  }

  private static SuperstructureRequest requestForCurrentMode(
      Supplier<SuperstructureRequest> requestSupplier) {
    SuperstructureRequest request = requestSupplier.get();
    return DriverStation.isAutonomous() ? request.asAuto() : request;
  }
}
