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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.ppp.PathBuilder;
import frc.robot.commands.Scoring.AutoCommands;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.simulation.SimulationVisualizer;
import frc.robot.subsystems.vision.VisConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
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
  private final Hood hood;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Wrist wrist;
  private final Climber climber;
  private final Vision vis;
  private SimulationVisualizer simvis;

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
                new VisionIOPhoton(VisConstants.leftPho, VisConstants.robotToCameraLeft),
                new VisionIOPhoton(VisConstants.rightPho, VisConstants.robotToCameraRight));

        hood = new Hood(new HoodIOReal());
        shooter = new Shooter(new ShooterIOReal());
        hopper = new Hopper(new HopperIOReal());
        intake = new Intake(new IntakeIOReal());
        wrist = new Wrist(new WristIOReal());
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

        hood = new Hood(new HoodIOSim());
        shooter = new Shooter(new ShooterIOSim());
        hopper = new Hopper(new HopperIOSim());
        intake = new Intake(new IntakeIOSim());
        wrist = new Wrist(new WristIOSim());
        climber = new Climber(new ClimberIOSim());

        simvis =
            new SimulationVisualizer(
                "Models",
                () -> Units.degreesToRadians(wrist.getAngle()),
                () -> Units.degreesToRadians(hood.getAngle()),
                () -> climber.getHeight());
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

        hood = new Hood(new HoodIO() {});
        shooter = new Shooter(new ShooterIO() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
        wrist = new Wrist(new WristIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto routines
    PathBuilder.configure(drive); // Add all subsystems as parameters later

    autoChooser = new LoggedDashboardChooser<>("Auto Choices"); // AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Right NZ 1.5 Center Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_midline_corner, Rotation2d.kCCW_90deg),
                        0.20),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.fromDegrees(105)),
                        0.20)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.right_midline_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.right_midline_corner, Rotation2d.kZero),
                        1,
                        1.5,
                        2),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(new Pose2d(2.975, 1.545, Rotation2d.kZero))),
                Commands.runOnce(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            Commands.runOnce(() -> PathBuilder.stopTarget())
                .andThen(AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist))));

    autoChooser.addOption(
        "Climb + Right NZ 1.5 Center Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_midline_corner, Rotation2d.kCCW_90deg),
                        0.20),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.fromDegrees(105)),
                        0.20)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.right_midline_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.right_midline_corner, Rotation2d.kZero),
                        1,
                        1.5,
                        2),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(new Pose2d(2.975, 1.545, Rotation2d.kZero))),
                Commands.runOnce(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            Commands.runOnce(() -> PathBuilder.stopTarget())
                .andThen(AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist)),
            Commands.sequence(
                Commands.runOnce(climber::raise, climber),
                PathBuilder.path(
                    new PathBuilder.Target(new Pose2d(2.975, 1.545, Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Tower.right_approach_pos, Rotation2d.kZero), 0.2),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Tower.right_align_pos, Rotation2d.kZero), 0.1)),
                Commands.runOnce(climber::lower, climber))));

    autoChooser.addOption(
        "Right NZ 1.5 Close Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.intake_right_midline_corner,
                            Rotation2d.kCCW_90deg),
                        0.20),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.intake_midline, Rotation2d.fromDegrees(105)),
                        0.20)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.intake_right_midline_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.intake_midline, Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.intake_right_midline_corner, Rotation2d.kZero),
                        1,
                        1.5,
                        2),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(new Pose2d(2.975, 1.545, Rotation2d.kZero))),
                Commands.runOnce(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            Commands.runOnce(() -> PathBuilder.stopTarget())
                .andThen(AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist))));

    autoChooser.addOption(
        "Right NZ 1 C Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_midline_corner, Rotation2d.kCCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.k180deg), 0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.middle_close_line, Rotation2d.kCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.right_close_corner, Rotation2d.kZero),
                        0.25)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.right_close_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.right_close_corner, Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero),
                        1),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            2.975,
                            1.545,
                            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                .minus(new Translation2d(2.975, 1.545))
                                .getAngle()),
                        0.5)),
                Commands.run(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_midline_corner, Rotation2d.kCCW_90deg),
                        0.4),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.kCCW_90deg), 0.4)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.right_midline_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0))))));

    autoChooser.addOption(
        "Optim. Right NZ 1 C Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kCCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_midline_corner,
                            Rotation2d.fromDegrees(105)),
                        0.4,
                        1.5),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.k180deg), 0.35),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.middle_close_line, Rotation2d.kCW_90deg),
                        0.4),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_close_corner, Rotation2d.kCW_90deg),
                        0.4)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.right_close_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_approach, Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            2.975,
                            1.545,
                            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                .minus(new Translation2d(2.975, 1.545))
                                .getAngle()),
                        0.5)),
                Commands.run(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            new Translation2d(
                                FieldConstants.FuelField.right_close_corner.getX()
                                    - Constants.Robot.B_WIDTH / 2,
                                FieldConstants.FuelField.right_close_corner.getY()),
                            Rotation2d.fromDegrees(105)),
                        0.4,
                        1.5),
                    new PathBuilder.Target(
                        new Pose2d(
                            new Translation2d(
                                FieldConstants.FuelField.middle_close_line.getX()
                                    - Constants.Robot.B_WIDTH / 2,
                                FieldConstants.FuelField.middle_close_line.getY()),
                            Rotation2d.k180deg),
                        0.4),
                    new PathBuilder.Target(
                        new Pose2d(
                            new Translation2d(
                                FieldConstants.FuelField.middle_close_line.getX()
                                    - (3 * Constants.Robot.B_WIDTH) / 2,
                                FieldConstants.FuelField.middle_close_line.getY()),
                            Rotation2d.kCW_90deg),
                        0.4),
                    new PathBuilder.Target(
                        new Pose2d(
                            new Translation2d(
                                FieldConstants.FuelField.right_close_corner.getX()
                                    - (3 * Constants.Robot.B_WIDTH) / 2,
                                FieldConstants.FuelField.right_close_corner.getY()),
                            Rotation2d.kCW_90deg),
                        0.4)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    new Translation2d(
                        FieldConstants.FuelField.right_close_corner.getX()
                            - Constants.Robot.B_WIDTH / 2,
                        FieldConstants.FuelField.right_close_corner.getY()),
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0))))));

    autoChooser.addOption(
        "Left NZ 1.5 Center Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_neutral_preentrance,
                            Rotation2d.kCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.left_midline_corner, Rotation2d.kCW_90deg),
                        0.20),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.fromDegrees(-105)),
                        0.20)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.left_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.left_midline_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.kCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.left_midline_corner, Rotation2d.kZero),
                        1,
                        1.5,
                        2),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(2.975, FieldConstants.field_width - 1.545, Rotation2d.kZero))),
                Commands.runOnce(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.left_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, FieldConstants.field_width - 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            Commands.runOnce(() -> PathBuilder.stopTarget())
                .andThen(AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist))));

    autoChooser.addOption(
        "Climb + Left NZ 1.5 Center Swipe",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_neutral_preentrance,
                            Rotation2d.kCW_90deg),
                        0.25),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.left_midline_corner, Rotation2d.kCW_90deg),
                        0.20),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.fromDegrees(-105)),
                        0.20)),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.left_trench_center,
                    1.5,
                    ScoringCommands.forceDown(wrist)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.left_midline_corner,
                    1,
                    Commands.run(() -> intake.intakeVolts(10.0)))),
            Commands.runOnce(() -> intake.stop()),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.field_center, Rotation2d.kCW_90deg)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.FuelField.left_midline_corner, Rotation2d.kZero),
                        1,
                        1.5,
                        2),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(2.975, FieldConstants.field_width - 1.545, Rotation2d.kZero))),
                Commands.runOnce(() -> intake.stop()),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.left_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, FieldConstants.field_width - 1.545),
                    0.1,
                    Commands.runOnce(() -> PathBuilder.stopTarget()))),
            Commands.runOnce(() -> PathBuilder.stopTarget())
                .andThen(AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist)),
            Commands.sequence(
                Commands.runOnce(climber::raise, climber),
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(2.975, FieldConstants.field_width - 1.545, Rotation2d.k180deg),
                        1,
                        0,
                        0.5),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Tower.left_approach_pos, Rotation2d.k180deg),
                        0.2),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Tower.left_intermediate_approach_pos,
                            Rotation2d.k180deg),
                        0.1),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Tower.left_inter_inter_approach_pos, Rotation2d.k180deg),
                        0.2),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Tower.left_align_pos, Rotation2d.k180deg), 0.05)),
                Commands.runOnce(climber::lower, climber))));

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
    pilot
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlip.apply(Rotation2d.kZero))),
                    drive)
                .ignoringDisable(true));

    Trigger driveInput =
        new Trigger(
            () ->
                (pilot.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0
                    || pilot.getCorrectedRight(Scale.LINEAR).getX() != 0.0
                    || pilot.getRightBumperButton().getAsBoolean()
                    || pilot.a().getAsBoolean()));

    driveInput.whileTrue(
        DriveCommands.joystickCombined(
            drive,
            () ->
                -pilot.getCorrectedLeft(Scale.SQUARED).getY()
                    * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () ->
                -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                    * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () ->
                -pilot.getCorrectedRight(Scale.SQUARED).getX()
                    * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () ->
                (pilot.a().getAsBoolean()
                    ? drive
                        .getPose()
                        .getTranslation()
                        .nearest(
                            List.of(
                                AllianceFlip.apply(FieldConstants.Bump.left_bump_alliance_entrance),
                                AllianceFlip.apply(
                                    FieldConstants.Bump.right_bump_alliance_entrance)))
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .minus(Rotation2d.fromDegrees(-6))
                    : AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getAngle()),
            () -> pilot.rightBumper().getAsBoolean() || pilot.a().getAsBoolean()));

    pilot.rightBumper().whileTrue(ScoringCommands.staticAim(drive, hood));
    pilot.a().whileTrue(ScoringCommands.passAim(hood));

    pilot
        .rightTrigger(0.3)
        .whileTrue(
            Commands.either(
                ScoringCommands.passShoot(shooter, hopper),
                // ScoringCommands.dataShoot(shooter, hopper),
                //  Commands.either(
                ScoringCommands.staticShoot(drive, shooter, hopper),
                // ScoringCommands.manualShots(hood, shooter, hopper),
                // () -> pilot.getRightBumperButton().getAsBoolean()),
                () -> pilot.a().getAsBoolean()));

    pilot
        .leftTrigger(0.3)
        .whileTrue(Commands.runEnd(() -> intake.intakeVolts(6.0), intake::stop, intake));

    pilot
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runEnd(() -> hopper.runHopperVolts(-6.0, -6.0), hopper::stop, hopper),
                Commands.runEnd(() -> intake.ejectVolts(6.0), intake::stop, intake)));

    pilot.y().toggleOnTrue(Commands.startEnd(climber::raise, climber::lower, climber));

    copilot
        .b()
        .whileTrue(
            Commands.runEnd(
                () -> climber.runClimberVolts(8 * -copilot.getLeftY(Scale.LINEAR)),
                climber::stop,
                climber));
    copilot
        .y()
        .whileTrue(
            Commands.runEnd(
                () -> wrist.runWristVolts(5 * -copilot.getLeftY(Scale.LINEAR)),
                wrist::stop,
                wrist));

    copilot.x().onTrue(ScoringCommands.downNoStall(wrist));
    copilot.a().onTrue(ScoringCommands.goodStow(wrist));

    copilot.getLeftBumperButton().whileTrue(ScoringCommands.shake(wrist));
    copilot.getLeftButton().onTrue(Commands.runOnce(climber::zero));
    copilot.getRightButton().onTrue(Commands.runOnce(wrist::zero));
    copilot.getUpButton().onTrue(Commands.runOnce(hood::setOffset));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // maybe making this easier for different positions in sim?!
  // idk man - ansh
  public void simReset() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      if (DriverStation.getLocation().getAsInt() == 3) {
        drive.setPose(
            new Pose2d(
                FieldConstants.Trench.right_trench_alliance_entrance.getX()
                    + Constants.Robot.B_LENGTH / 2,
                FieldConstants.Trench.right_trench_alliance_entrance.getY(),
                Rotation2d.kCCW_90deg));
      } else if (DriverStation.getLocation().getAsInt() == 2) {
        drive.setPose(
            new Pose2d(
                new Translation2d(3.57, Units.inchesToMeters(317.69) / 2), Rotation2d.kZero));
      } else if (DriverStation.getLocation().getAsInt() == 1) {
        drive.setPose(
            new Pose2d(
                FieldConstants.Trench.left_trench_alliance_entrance.getX()
                    + Constants.Robot.B_LENGTH / 2,
                FieldConstants.Trench.left_trench_alliance_entrance.getY(),
                Rotation2d.kCW_90deg));
      }
    } else if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      if (DriverStation.getLocation().getAsInt() == 3) {
        drive.setPose(
            AllianceFlip.apply(
                new Pose2d(
                    FieldConstants.Trench.right_trench_alliance_entrance.getX()
                        + Constants.Robot.B_LENGTH / 2,
                    FieldConstants.Trench.right_trench_alliance_entrance.getY(),
                    Rotation2d.kCCW_90deg)));
      } else if (DriverStation.getLocation().getAsInt() == 2) {
        drive.setPose(
            AllianceFlip.apply(
                new Pose2d(
                    new Translation2d(3.57, Units.inchesToMeters(317.69) / 2), Rotation2d.kZero)));
      } else if (DriverStation.getLocation().getAsInt() == 1) {
        drive.setPose(
            AllianceFlip.apply(
                new Pose2d(
                    FieldConstants.Trench.left_trench_alliance_entrance.getX()
                        + Constants.Robot.B_LENGTH / 2,
                    FieldConstants.Trench.left_trench_alliance_entrance.getY(),
                    Rotation2d.kCW_90deg)));
      }
    }
  }

  public void periodic() {
    Logger.recordOutput("Drive/Angle", drive.getPose().getRotation().getDegrees());
    Logger.recordOutput(
        "Drive/Setpoint", Constants.DriveConstants.ANGLE_PID.getSetpoint().position);
    Logger.recordOutput("Drive/At Goal?", Constants.DriveConstants.ANGLE_PID.atGoal());

    Logger.recordOutput(
        "Drive/Distance From Hub",
        FieldConstants.Hub.hub_center_2d.getDistance(drive.getPose().getTranslation()));
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.Robot.currentMode != Constants.Mode.SIM) return;

    simvis.update();

    Logger.recordOutput("FieldSimulation/RobotPosition", drive.getPose());
  }
}
