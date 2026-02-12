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
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Launcher.Hood.HoodIO;
import frc.robot.subsystems.Launcher.Hood.HoodIOReal;
import frc.robot.subsystems.Launcher.Hood.HoodIOSim;
import frc.robot.subsystems.Launcher.Launcher;
import frc.robot.subsystems.Launcher.Shooter.ShooterIO;
import frc.robot.subsystems.Launcher.Shooter.ShooterIOReal;
import frc.robot.subsystems.Launcher.Shooter.ShooterIOSim;
import frc.robot.subsystems.Loader.Intake.IntakeIO;
import frc.robot.subsystems.Loader.Intake.IntakeIOReal;
import frc.robot.subsystems.Loader.Intake.IntakeIOSim;
import frc.robot.subsystems.Loader.Loader;
import frc.robot.subsystems.Loader.Wrist.WristIO;
import frc.robot.subsystems.Loader.Wrist.WristIOReal;
import frc.robot.subsystems.Loader.Wrist.WristIOSim;
import frc.robot.subsystems.Transfer.Hopper.HopperIO;
import frc.robot.subsystems.Transfer.Hopper.HopperIOReal;
import frc.robot.subsystems.Transfer.Hopper.HopperIOSim;
import frc.robot.subsystems.Transfer.Indexer.IndexerIO;
import frc.robot.subsystems.Transfer.Indexer.IndexerIOReal;
import frc.robot.subsystems.Transfer.Indexer.IndexerIOSim;
import frc.robot.subsystems.Transfer.Transfer;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Launcher launcher;
  private final Loader loader;
  private final Transfer transfer;
  private final Climber climber;
  private final PIDTuning pidTuner;
  private final Vision vis;

  // Controller
  private final CSP_Controller pilot = new CSP_Controller(Constants.Controller.kPilotPort);
  private final CSP_Controller copilot = new CSP_Controller(Constants.Controller.kCopilotPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.Robot.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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

        launcher = new Launcher(new ShooterIOReal(), new HoodIOReal());
        loader = new Loader(new IntakeIOReal(), new WristIOReal());
        transfer = new Transfer(new HopperIOReal(), new IndexerIOReal());
        climber = new Climber(new ClimberIOReal());

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

        launcher = new Launcher(new ShooterIOSim(), new HoodIOSim());
        loader = new Loader(new IntakeIOSim(), new WristIOSim());
        transfer = new Transfer(new HopperIOSim(), new IndexerIOSim());
        climber = new Climber(new ClimberIOSim());
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

        launcher = new Launcher(new ShooterIO() {}, new HoodIO() {});
        loader = new Loader(new IntakeIO() {}, new WristIO() {});
        transfer = new Transfer(new HopperIO() {}, new IndexerIO() {});
        climber = new Climber(new ClimberIO() {});
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
      case SHOOTER:
        pidTuner =
            new PIDTuning(
                "Shooter",
                () -> {
                  return 0;
                },
                (set) -> launcher.runShooter(set),
                launcher::updateShooterPID);
        break;
      case HOOD:
        pidTuner =
            new PIDTuning(
                "Hood",
                () -> {
                  return 0;
                },
                (set) -> launcher.setHood(Rotation2d.fromRadians(set)),
                launcher::updateHoodPID);
        break;
      case WRIST:
        pidTuner =
            new PIDTuning(
                "Intake Wrist",
                () -> {
                  return 0;
                },
                (set) -> loader.setWrist(Rotation2d.fromRadians(set)),
                loader::updateWristPID);
        break;

      case NONE:
      default:
        pidTuner = new PIDTuning();
    }

    // Set up auto routines
    PathBuilder.configure(drive); // Add all subsystems as parameters later

    // Set global constraints before creating any paths
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            Constants.Drive.DRIVE_MAXVEL, // maxVelocityMetersPerSec
            Constants.Drive.DRIVE_MAXACC, // maxAccelerationMetersPerSec2
            Constants.Drive.ANGLE_MAXVEL * 180 / Math.PI, // maxVelocityDegPerSec
            Constants.Drive.ANGLE_MAXACC * 180 / Math.PI, // maxAccelerationDegPerSec2
            0.03, // endTranslationToleranceMeters
            2.0, // endRotationToleranceDeg
            0.2 // intermediateHandoffRadiusMeters
            ));

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

    autoChooser.addOption(
        "Sigma",
        PathBuilder.pathBuilder.build(
            new Path(
                new Path.Waypoint(
                    FieldConstants.Trench.left_trench_alliance_preentrance, Rotation2d.kZero),
                new Path.Waypoint(
                    FieldConstants.Trench.left_trench_alliance_entrance, Rotation2d.kZero))));

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
                PathBuilder.createPath(
                    new Pose2d(
                        FieldConstants.Trench.right_trench_neutral_preentrance,
                        Rotation2d.kCCW_90deg),
                    5))
            .andThen(
                PathBuilder.createPath(
                    new Pose2d(
                        FieldConstants.FuelField.right_close_corner_approach,
                        Rotation2d.kCCW_90deg),
                    5))
            .andThen(
                PathBuilder.createPath(
                    new Pose2d(
                        FieldConstants.FuelField.left_close_corner_approach, Rotation2d.kCCW_90deg),
                    5))
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
            .andThen(
                PathBuilder.createPath(
                    new Pose2d(FieldConstants.Tower.left_far_corner, Rotation2d.k180deg), 0)));

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
                                  AllianceFlip.flipX(FieldConstants.Hub.hub_center_2d.getX())
                                      - drive.getPose().getTranslation().getX(),
                                  AllianceFlip.flipY(FieldConstants.Hub.hub_center_2d.getY())
                                      - drive.getPose().getTranslation().getY(),
                                  Units.inchesToMeters(72 - 20)),
                              new Translation2d(
                                      drive.getChassisSpeeds().vxMetersPerSecond,
                                      drive.getChassisSpeeds().vyMetersPerSecond)
                                  .rotateBy(drive.getRotation()));
                      if (result.getY() == -Math.PI / 2) {
                        return AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
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
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.left_trench_alliance_preentrance,
                                Rotation2d.kZero)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .onFalse(Commands.runOnce(drive::stopWithX, drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void simReset() {
    drive.setPose(new Pose2d(new Translation2d(3.54, 2), Rotation2d.kZero));
  }

  public void periodic() {
    if (Constants.Robot.tuningMode != Constants.PIDTuning.NONE) pidTuner.updateLoop();

    Logger.recordOutput("Drive/Angle At Setpoint?", Constants.Drive.ANGLE_PID.atGoal());

    // Logger.recordOutput("State/Robot Mode", Constants.Robot.robotMode);

    // testing placeholder
    // if (AllianceFlip.flipX(drive.getPose().getX())
    //     < FieldConstants.alliance_zone_x - Constants.Robot.B_LENGTH) {
    //   Constants.Robot.robotMode = Constants.RobotMode.SHOOT;
    // } else {
    //   Constants.Robot.robotMode = Constants.RobotMode.NONE;
    // }
  }
}
