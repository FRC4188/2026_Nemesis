// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.pidtuning.PIDTuning;
import frc.robot.CSPLib.ppp.PathBuilder;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 *  * Change Log
 * - Added changable AutoBuilder configurations based on shooting mode
 * - Renamed "instantniate" to "configure" in PathBuilder
 * - Temporary hueristic of shooting mode located bottom of Robot Conatiner
 * - Removed orientation angle from Drive Class (put it somewhere else)
 * 
 * TODO: PPP to do list for Priyanshu and Ansh
 * 
 * 1. Add Waypoints and Events in PathBuilder
 * - Chaining paths is slow and clunky to combine with external commands
 * - Better if we use waypoints and events like in PP (those classes exist in the library)
 * - Have common waypoints in constants or something (Trench travel, Fuel Gathering)
 * - Try not to rely on AD star besides Hueristic
 * 
 * 2. Clean up Drive
 * - Try to add the least amount of methods possible to the subsystem
 * - Any command related stuff should be outside: ie your rotation huerisitc
 * 
 * 3. Hueristic
 * - Add in PathBuilder or other class (not drive)
 * - Add Boolean supplier to detect if robot is blocked (this will run in the "until" part of auto)
 * - Add calcation method to detect where obstacle is and place it
 * - Then use AD star command to chain to the next Waypoint from Path Builder and continue as normal
 * 
 * 4. Other
 * - Add potential starting poses for simulation into FieldConstants
 * - Remove giant comment blocks
 * - Fix licensing so its proper
 * 
 */

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final PIDTuning pidTuner;

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
    PathBuilder.configure(drive);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices"); // AutoBuilder.buildAutoChooser());

    // autoChooser.addOption(
    //     "Best Auto",
    //     Commands.runOnce(
    //             () ->
    //                 drive.setRotationPoint(
    //                     // () ->
    //                     //     FieldConstants.Hub.hub_center_2d
    //                     //         .minus(drive.getPose().getTranslation())
    //                     //         .getAngle()))
    //                     () -> new Rotation2d()))
    //         .andThen(
    //             AutoBuilder.pathfindToPose(
    //                 new Pose2d(
    //                     FieldConstants.Trench.left_trench_alliance_entrance, new
    // Rotation2d(Math.PI / 2)),
    //                 new PathConstraints(
    //                     3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
    //                 0.0))
    //         .beforeStarting(() -> drive.angleController.reset(drive.getRotation().getRadians()))
    //         .until(() ->
    // drive.getPose().getTranslation().getDistance(FieldConstants.Trench.left_trench_alliance_entrance) <= Units.inchesToMeters(5.0))
    //         .finallyDo(() -> drive.stop())
    //         );

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
                () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d)).andThen(
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -pilot.getLeftY(), () -> -pilot.getLeftX(), () -> -pilot.getRightX()));

    // Lock to 0° when A button is held
    pilot
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -pilot.getLeftY(),
                () -> -pilot.getLeftX(),
                () ->
                    FieldConstants.Hub.hub_center_2d
                        .minus(drive.getPose().getTranslation())
                        .getAngle()));

    // Switch to X pattern when X button is pressed
    pilot.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    pilot
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simReset() {
    drive.setPose(new Pose2d(new Translation2d(1, 1), Rotation2d.fromDegrees(60)));
  }

  public void periodic() {
    if (Constants.Robot.tuningMode != Constants.PIDTuning.NONE) pidTuner.updateLoop();

    // testing placeholder
    if (AllianceFlip.flipX(drive.getPose().getX()) < FieldConstants.alliance_zone_x - Constants.Robot.B_LENGTH) {
      Constants.Robot.robotMode = Constants.RobotMode.SHOOT;
    } else {
      Constants.Robot.robotMode = Constants.RobotMode.NONE;
    }
  }
}
