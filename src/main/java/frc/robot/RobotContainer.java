// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.pidtuning.PIDTuning;
import frc.robot.CSPLib.ppp.PathBuilder;
import frc.robot.CSPLib.util.ProjMath;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.VisConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * * Change Log - Added changable AutoBuilder configurations based on shooting mode - Renamed
 * "instantniate" to "configure" in PathBuilder - Temporary hueristic of shooting mode located
 * bottom of Robot Conatiner - Removed orientation angle from Drive Class (put it somewhere else)
 *
 * <p>TODO: PPP to do list for Priyanshu and Ansh
 *
 * <p>1. Add Waypoints and Events in PathBuilder - Chaining paths is slow and clunky to combine with
 * external commands - Better if we use waypoints and events like in PP (those classes exist in the
 * library) - Have common waypoints in constants or something (Trench travel, Fuel Gathering) - Try
 * not to rely on AD star besides Hueristic
 *
 * <p>2. Clean up Drive - Try to add the least amount of methods possible to the subsystem - Any
 * command related stuff should be outside: ie your rotation huerisitc
 *
 * <p>3. Hueristic - Add in PathBuilder or other class (not drive) - Add Boolean supplier to detect
 * if robot is blocked (this will run in the "until" part of auto) - Add calcation method to detect
 * where obstacle is and place it - Then use AD star command to chain to the next Waypoint from Path
 * Builder and continue as normal
 *
 * <p>4. Other - Add potential starting poses for simulation into FieldConstants - Remove giant
 * comment blocks - Fix licensing so its proper
 * * Change Log - Added changable AutoBuilder configurations based on shooting mode - Renamed
 * "instantniate" to "configure" in PathBuilder - Temporary hueristic of shooting mode located
 * bottom of Robot Conatiner - Removed orientation angle from Drive Class (put it somewhere else)
 *
 * <p>TODO: PPP to do list for Priyanshu and Ansh
 *
 * <p>1. Add Waypoints and Events in PathBuilder - Chaining paths is slow and clunky to combine with
 * external commands - Better if we use waypoints and events like in PP (those classes exist in the
 * library) - Have common waypoints in constants or something (Trench travel, Fuel Gathering) - Try
 * not to rely on AD star besides Hueristic
 *
 * <p>2. Clean up Drive - Try to add the least amount of methods possible to the subsystem - Any
 * command related stuff should be outside: ie your rotation huerisitc
 *
 * <p>3. Hueristic - Add in PathBuilder or other class (not drive) - Add Boolean supplier to detect
 * if robot is blocked (this will run in the "until" part of auto) - Add calcation method to detect
 * where obstacle is and place it - Then use AD star command to chain to the next Waypoint from Path
 * Builder and continue as normal
 *
 * <p>4. Other - Add potential starting poses for simulation into FieldConstants - Remove giant
 * comment blocks - Fix licensing so its proper
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final PIDTuning pidTuner;
  private final Vision vis;

  // Controller
  private final CSP_Controller pilot = new CSP_Controller(Constants.Controller.kPilotPort);
  private final CSP_Controller copilot = new CSP_Controller(Constants.Controller.kCopilotPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.Robot.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vis =
            new Vision(
                drive::accept,
                new VisionIOPhoton(VisConstants.frontPho, VisConstants.robotToCamera0),
                new VisionIOPhoton(VisConstants.backPho, VisConstants.robotToCamera2));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vis = new Vision(drive::accept, new VisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vis = new Vision(drive::accept, new VisionIO() {});

        break;
    }

    switch (Constants.Robot.tuningMode) {
      case DRIVE_MOD:
        pidTuner = new PIDTuning("Drive Modules", () -> 0, (set) -> {}, drive::updateDrivePID);
        break;
      case TURN_MOD:
        pidTuner = new PIDTuning("Turn Modules", () -> 0, (set) -> {}, drive::updateTurnPID);
        break;
      case ANGLE:
        pidTuner =
            new PIDTuning(
                "Angle Controller",
                () -> drive.getPose().getRotation().getRadians(),
                (set) -> {},
                Constants.Drive::updateAnglePID);
        break;
      case NONE:
      default:
        pidTuner = new PIDTuning();
    }

    // Set up auto routines
    PathBuilder.configure(drive); // Add all subsystems as parameters later
    autoChooser = new LoggedDashboardChooser<>("Auto Choices"); // AutoBuilder.buildAutoChooser());

    // autoChooser.addOption(
    //     "Test PathBuilder",
    //     PathBuilder.generalAuton(
    //         new Pose2d(FieldConstants.Trench.right_trench_alliance_preentrance, new
    // Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.right_trench_alliance_entrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.right_trench_neutral_entrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.right_trench_neutral_preentrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.FuelField.right_close_corner, new Rotation2d()),
    //         new Pose2d(FieldConstants.FuelField.right_midline_corner, new Rotation2d()),
    //         new Pose2d(FieldConstants.FuelField.left_midline_corner, new Rotation2d()),
    //         new Pose2d(FieldConstants.FuelField.left_close_corner, new Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.left_trench_neutral_preentrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.left_trench_neutral_entrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.left_trench_alliance_entrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.Trench.left_trench_alliance_preentrance, new Rotation2d()),
    //         new Pose2d(FieldConstants.Tower.left_far_corner, new Rotation2d())));

    autoChooser.addOption(
        "PPP",
        Commands.runOnce(
                () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
            .andThen(PathBuilder.createPath(FieldConstants.Trench.left_trench_center, 5.0))
            .andThen(Commands.runOnce(() -> PathBuilder.stopTarget()))
            .andThen(PathBuilder.createPath(FieldConstants.FuelField.right_midline_corner, 0.0)));

    autoChooser.addOption(
        "TestChain",
        Commands.runOnce(
                () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
            .andThen(
                PathBuilder.createPath(
                    FieldConstants.FuelField.right_midline_corner, new Translation2d(1, 1))));

    autoChooser.addOption(
        "All Together Now",
        Commands.runOnce(
                () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
            .andThen(PathBuilder.createPath(FieldConstants.Trench.left_trench_alliance_preentrance))
            .andThen(() -> PathBuilder.targetRotation(() -> Rotation2d.kZero))
            .andThen(
                () -> PathBuilder.createPath(FieldConstants.Trench.left_trench_alliance_entrance))
            .andThen(
                PathBuilder.createPath(
                    new Pose2d(
                        FieldConstants.Trench.left_trench_neutral_entrance, new Rotation2d(0))))
            .andThen(PathBuilder.createPath(FieldConstants.FuelField.right_midline_corner)));

    // .andThen(
    //     PathBuilder.mergeToKnownPath(
    //         new PathPlannerPath(
    //             FieldConstants.Tower.left_approach,
    //             PathBuilder.getConstraints(),
    //             null,
    //             new GoalEndState(0.0, Rotation2d.k180deg)))));

    autoChooser.addOption(
        "TO THE RIGHT, TO THE LEFT",
        Commands.runOnce(
                () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
            .andThen(PathBuilder.createPath(FieldConstants.Tower.right_far_corner, 5))
            .andThen(
                PathBuilder.createPath(FieldConstants.Trench.right_trench_alliance_preentrance, 5))
            .andThen(Commands.runOnce(() -> PathBuilder.stopTarget()))
            .andThen(
                PathBuilder.createPath(new Pose2d(FieldConstants.Trench.right_trench_neutral_preentrance, Rotation2d.kCCW_90deg), 5))
            .andThen(
                PathBuilder.createPath(new Pose2d(FieldConstants.FuelField.right_close_corner_approach, Rotation2d.kCCW_90deg), 5))
            .andThen(PathBuilder.createPath(new Pose2d(FieldConstants.FuelField.left_close_corner_approach, Rotation2d.kCCW_90deg), 5))
            .andThen(Commands.runOnce(() -> PathBuilder.stopTarget()))
            .andThen(
                PathBuilder.createPath(FieldConstants.Trench.left_trench_neutral_preentrance, 5))
            .andThen(
                PathBuilder.createPath(FieldConstants.Trench.left_trench_alliance_preentrance, 5))
            .andThen(
                Commands.runOnce(
                    () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d)))
            .andThen(PathBuilder.createPath(FieldConstants.Depot.left_far_corner, 0))
            .andThen(Commands.waitSeconds(5))
            .andThen(Commands.runOnce(() -> PathBuilder.stopTarget()))
            .andThen(PathBuilder.createPath(new Pose2d(FieldConstants.Tower.left_far_corner, Rotation2d.k180deg), 0)));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    Trigger driveInput =
        new Trigger(
            () ->
                (pilot.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0
                    || pilot.getCorrectedRight(Scale.LINEAR).getX() != 0.0));

    driveInput
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () ->
                    -pilot.getCorrectedLeft(Scale.SQUARED).getY()
                        * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
                () ->
                    -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                        * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
                () ->
                    -pilot.getCorrectedRight(Scale.SQUARED).getX()
                        * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0)))
        .onFalse(Commands.runOnce(drive::stop, drive));

    pilot
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () ->
                        -pilot.getCorrectedLeft(Scale.SQUARED).getY()
                            * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
                    () ->
                        -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                            * (pilot.rightBumper().getAsBoolean() ? 0.5 : 1.0),
                    () -> {
                      Rotation3d result =
                          ProjMath.movingShot(
                              7,
                              new Translation3d(
                                  FieldConstants.Hub.hub_center_2d.getX()
                                      - drive.getPose().getTranslation().getX(),
                                  FieldConstants.Hub.hub_center_2d.getY()
                                      - drive.getPose().getTranslation().getY(),
                                  Units.inchesToMeters(72 - 20)),
                              new Translation2d(
                                      drive.getChassisSpeeds().vxMetersPerSecond,
                                      drive.getChassisSpeeds().vyMetersPerSecond)
                                  .rotateBy(drive.getRotation()));
                      if (result.getY() == -Math.PI / 2) {
                        return FieldConstants.Hub.hub_center_2d
                            .minus(drive.getPose().getTranslation())
                            .getAngle();
                      } else {
                        return Rotation2d.fromRadians(result.getZ());
                      }
                    })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(Commands.runOnce(drive::stopWithX, drive));

    pilot
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    pilot
        .x()
        .and(pilot.leftBumper())
        .onTrue(Commands.runOnce(() -> drive.acceptVision(true), drive));

    pilot
        .y()
        .and(pilot.leftBumper())
        .onTrue(Commands.runOnce(() -> drive.acceptVision(false), drive));

    pilot
        .b()
        .whileTrue(
            new DriveToPose(
                    drive,
                    () ->
                        AllianceFlip.flipDS(
                            new Pose2d(
                                FieldConstants.Trench.left_trench_alliance_preentrance,
                                Rotation2d.kZero)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(Commands.runOnce(drive::stopWithX, drive));

    // // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive, () -> -pilot.getCorrectedLeft(Scale.SQUARED).getY(), () -> -pilot.getLeftX(),
    // () -> -pilot.getRightX()));

    // // Lock to 0° when A button is held
    // pilot
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -pilot.getLeftY(),
    //             () -> -pilot.getLeftX(),
    //             () ->
    //                 FieldConstants.Hub.hub_center_2d
    //                     .minus(drive.getPose().getTranslation())
    //                     .getAngle()));

    // // Switch to X pattern when X button is pressed
    // pilot.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // pilot
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simReset() {
    drive.setPose(new Pose2d(new Translation2d(3.54, 2), Rotation2d.kZero));
  }

  public void periodic() {
    if (Constants.Robot.tuningMode != Constants.PIDTuning.NONE) pidTuner.updateLoop();

    Logger.recordOutput("State/Robot Mode", Constants.Robot.robotMode);

    // testing placeholder
    if (AllianceFlip.flipX(drive.getPose().getX())
        < FieldConstants.alliance_zone_x - Constants.Robot.B_LENGTH) {
      Constants.Robot.robotMode = Constants.RobotMode.SHOOT;
    } else {
      Constants.Robot.robotMode = Constants.RobotMode.NONE;
    }
  }
}
