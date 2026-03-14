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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.ppp.PathBuilder;
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

    autoChooser.addOption("help me please reagen", ScoringCommands.staticAim(drive, hood));

    autoChooser.addOption(
        "BEGONE WITCHES",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.interpolateTimedPath(
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_center, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                new Rotation2d())))
                    .andThen(
                        PathBuilder.interpolateTimedPath(
                            PathBuilder.scaleSpeeds(0.3),
                            AllianceFlip.apply(
                                new Pose2d(
                                    FieldConstants.FuelField.right_close_corner, new Rotation2d())),
                            AllianceFlip.apply(
                                new Pose2d(FieldConstants.field_center, new Rotation2d())))),
                PathBuilder.triggerWhenClose(
                    AllianceFlip.apply(FieldConstants.Trench.right_trench_center),
                    2.5,
                    Commands.runOnce(
                        () -> {
                          PathBuilder.targetRotation(() -> AllianceFlip.apply(Rotation2d.kZero));
                        })),
                PathBuilder.triggerWhenClose(
                    AllianceFlip.apply(FieldConstants.FuelField.right_close_corner),
                    1,
                    Commands.runOnce(
                            () -> {
                              PathBuilder.targetRotation(() -> Rotation2d.kCCW_90deg);
                            })
                        .andThen(Commands.run(() -> intake.intakeVolts(10.0))))),
            Commands.deadline(
                PathBuilder.interpolateTimedPath(
                    AllianceFlip.apply(new Pose2d(FieldConstants.field_center, new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(FieldConstants.FuelField.right_close_corner, new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_alliance_preentrance,
                            new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(new Translation2d(2.975, 1.545), new Rotation2d()))),
                PathBuilder.triggerWhenClose(
                    AllianceFlip.apply(FieldConstants.FuelField.right_midline_corner),
                    1.5,
                    Commands.runOnce(
                            () -> {
                              PathBuilder.targetRotation(
                                  () -> AllianceFlip.apply(Rotation2d.kZero));
                            })
                        .andThen(Commands.run(() -> intake.intakeVolts(0)))),
                PathBuilder.triggerWhenClose(
                    AllianceFlip.apply(new Translation2d(2.975, 1.545)),
                    0.5,
                    Commands.runOnce(
                            () -> {
                              PathBuilder.targetTranslation(
                                  () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d));
                            })
                        .andThen(Commands.run(() -> intake.intakeVolts(0))))),
            Commands.runOnce(() -> PathBuilder.stopTarget()),
            Commands.parallel(
                    DriveCommands.autonAtAngle(
                        drive,
                        () ->
                            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                .minus(drive.getPose().getTranslation())
                                .getAngle()),
                    Commands.runEnd(() -> intake.intakeVolts(1.5), () -> intake.stop())
                        .withTimeout(1),
                    ScoringCommands.staticAim(drive, hood),
                    new WaitCommand(0.1)
                        .andThen(new WaitUntilCommand(() -> shooter.atGoal()))
                        .andThen(ScoringCommands.staticShoot(drive, shooter, hopper)))
                .withTimeout(4.0)));

    autoChooser.addOption(
        "witches gone",
        Commands.sequence(
            Commands.deadline(
                PathBuilder.interpolateTimedPath(
                        AllianceFlip.apply(
                            new Pose2d(FieldConstants.Trench.left_trench_center, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.left_trench_neutral_preentrance,
                                new Rotation2d())))
                    .andThen(
                        PathBuilder.interpolateTimedPath(
                            PathBuilder.scaleSpeeds(0.5),
                            AllianceFlip.apply(
                                new Pose2d(
                                    FieldConstants.FuelField.left_close_corner, new Rotation2d())),
                            AllianceFlip.apply(
                                new Pose2d(
                                    FieldConstants.field_center, Rotation2d.fromDegrees(120))))),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.left_trench_center,
                    2.5,
                    Commands.runOnce(
                        () -> {
                          PathBuilder.targetRotation(() -> Rotation2d.kZero);
                        })),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.left_close_corner,
                    1,
                    Commands.runOnce(
                            () -> {
                              PathBuilder.stopTarget();
                            })
                        .andThen(Commands.run(() -> intake.intakeVolts(10.0))))),
            Commands.deadline(
                PathBuilder.interpolateTimedPath(
                    AllianceFlip.apply(new Pose2d(FieldConstants.field_center, new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(FieldConstants.FuelField.left_close_corner, new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_neutral_preentrance,
                            new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_alliance_preentrance,
                            new Rotation2d())),
                    AllianceFlip.apply(
                        new Pose2d(new Translation2d(2.975, 1.545), new Rotation2d()))),
                PathBuilder.triggerWhenClose(
                    FieldConstants.FuelField.left_midline_corner,
                    1.5,
                    Commands.runOnce(
                            () -> {
                              PathBuilder.targetRotation(() -> Rotation2d.kZero);
                            })
                        .andThen(Commands.run(() -> intake.intakeVolts(0)))),
                PathBuilder.triggerWhenClose(
                    new Translation2d(2.975, 1.545),
                    0.5,
                    Commands.runOnce(
                            () -> {
                              PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d);
                            })
                        .andThen(Commands.run(() -> intake.intakeVolts(0))))),
            Commands.runOnce(() -> PathBuilder.stopTarget()),
            Commands.parallel(
                    DriveCommands.autonAtAngle(
                        drive,
                        () ->
                            AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                .minus(drive.getPose().getTranslation())
                                .getAngle()
                                .minus(Rotation2d.fromDegrees(-6))),
                    Commands.runEnd(() -> intake.intakeVolts(1.5), () -> intake.stop())
                        .withTimeout(1),
                    ScoringCommands.staticAim(drive, hood),
                    new WaitCommand(0.1)
                        // .andThen(new WaitUntilCommand(() -> hood.atGoal()))
                        .andThen(ScoringCommands.staticShoot(drive, shooter, hopper)))
                .withTimeout(4.0)));

    autoChooser.addOption(
        "i hate code",
        Commands.sequence(
            Commands.parallel(
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_center,
                    2.5,
                    Commands.runOnce(() -> PathBuilder.targetRotation(() -> Rotation2d.kZero))),
                PathBuilder.interpolateTimedPath(
                        // AllianceFlip.apply(
                        //     new Pose2d(FieldConstants.right_alliance_shoot, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_alliance_preentrance,
                                new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                new Rotation2d())))
                    .andThen(Commands.runOnce(() -> PathBuilder.stopTarget()))),
            Commands.runOnce(
                () -> {
                  PathBuilder.targetRotation(() -> Rotation2d.kCCW_90deg);
                }),
            Commands.deadline(
                    PathBuilder.interpolateTimedPath(
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.FuelField.right_close_corner, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(FieldConstants.field_center, new Rotation2d()))),
                    Commands.run(() -> intake.intakeVolts(0.7)))
                .andThen(
                    Commands.runOnce(() -> PathBuilder.targetRotation(() -> Rotation2d.kZero))),
            Commands.parallel(
                    PathBuilder.triggerWhenClose(
                        FieldConstants.Trench.right_trench_center,
                        2.5,
                        Commands.runOnce(() -> PathBuilder.targetRotation(() -> Rotation2d.kZero))),
                    PathBuilder.interpolateTimedPath(
                        AllianceFlip.apply(
                            new Pose2d(FieldConstants.field_center, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.FuelField.right_midline_corner, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_alliance_preentrance,
                                new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(FieldConstants.right_alliance_shoot, new Rotation2d())),
                        AllianceFlip.apply(
                            new Pose2d(new Translation2d(2.975, 1.545), new Rotation2d()))))
                .andThen(
                    Commands.parallel(
                        DriveCommands.autonAtAngle(
                            drive,
                            () ->
                                AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                                    .minus(drive.getPose().getTranslation())
                                    .getAngle()
                                    .minus(Rotation2d.fromDegrees(-6))),
                        ScoringCommands.staticAim(drive, hood))),
            new WaitUntilCommand(() -> hood.atGoal())
                .andThen(ScoringCommands.staticShoot(drive, shooter, hopper).withTimeout(5))));

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
                ScoringCommands.staticShoot(drive, shooter, hopper),
                () -> pilot.a().getAsBoolean()));

    pilot
        .leftTrigger(0.3)
        .whileTrue(Commands.runEnd(() -> intake.intakeVolts(6.0), intake::stop, intake));

    pilot
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runEnd(() -> hopper.runHopperVolts(-6.0), hopper::stop, hopper),
                Commands.runEnd(() -> intake.ejectVolts(6.0), intake::stop, intake)));

    pilot.y().toggleOnTrue(Commands.startEnd(climber::raise, climber::lower, climber));
    // pilot.x().onTrue(ScoringCommands.goToClimb(drive, climber));

    copilot
        .b()
        .whileTrue(
            Commands.runEnd(
                () -> climber.runClimberVolts(8 * -copilot.getLeftY(Scale.LINEAR)),
                climber::stop,
                climber));

    copilot
        .x()
        .whileTrue(
            Commands.runEnd(
                () -> hood.runHoodVolts(-copilot.getLeftY(Scale.LINEAR)), hood::stop, hood));
    copilot
        .y()
        .whileTrue(
            Commands.runEnd(
                () -> wrist.runWristVolts(3 * -copilot.getLeftY(Scale.LINEAR)),
                wrist::stop,
                wrist));

    copilot.a().whileTrue(ScoringCommands.shake(wrist));

    copilot.getLeftBumperButton().onTrue(ScoringCommands.downWoStall(wrist));
    copilot.getRightBumperButton().onTrue(Commands.runOnce(wrist::stow, wrist));

    copilot.getLeftButton().onTrue(Commands.runOnce(climber::zero));
    // copilot.getRightButton().onTrue(Commands.runOnce(wrist::zero));
    copilot.getUpButton().onTrue(Commands.runOnce(hood::zero));

    copilot.rightTrigger(0.3).onTrue(Commands.runOnce(() -> drive.acceptVision(true)));
    copilot.leftTrigger(0.3).onTrue(Commands.runOnce(() -> drive.acceptVision(false)));
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
                Rotation2d.kZero));
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
                Rotation2d.kZero));
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
                    Rotation2d.kZero)));
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
                    Rotation2d.kZero)));
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
