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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.ppp.NodePathGenerator;
import frc.robot.CSPLib.ppp.PathBuilder;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberIOSim;
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

  // don't remove the suppresswarnings or else it will be annoying
  // trust me
  @SuppressWarnings("unchecked")
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
                () -> wrist.getAngle(),
                () -> hood.getShotAngle(),
                () -> climber.getHeightRots());
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

    // Set global constraints before creating any paths
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            Constants.DriveConstants.DRIVE_MAXVEL, // maxVelocityMetersPerSec
            Constants.DriveConstants.DRIVE_MAXACC, // maxAccelerationMetersPerSec2
            Constants.DriveConstants.ANGLE_MAXVEL * 180 / Math.PI, // maxVelocityDegPerSec
            Constants.DriveConstants.ANGLE_MAXACC * 180 / Math.PI, // maxAccelerationDegPerSec2
            0.03, // endTranslationToleranceMeters
            2.0, // endRotationToleranceDeg
            0.2 // intermediateHandoffRadiusMeters
            ));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices"); // AutoBuilder.buildAutoChooser());

    // autoChooser.addOption(
    //     "PPP",
    //     Commands.runOnce(
    //             () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
    //         .andThen(PathBuilder.createPath(FieldConstants.Trench.left_trench_center, 5.0))
    //         .andThen(Commands.runOnce(() -> PathBuilder.stopTarget()))
    //         .andThen(PathBuilder.createPath(FieldConstants.FuelField.right_midline_corner,
    // 0.0)));

    // autoChooser.addOption(
    //     "TestChain",
    //     Commands.runOnce(
    //             () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
    //         .andThen(
    //             PathBuilder.createPath(
    //                 FieldConstants.FuelField.right_midline_corner, new Translation2d(1, 1))));

    // autoChooser.addOption(
    //     "All Together Now",
    //     Commands.runOnce(
    //             () -> PathBuilder.targetTranslation(() -> FieldConstants.Hub.hub_center_2d))
    //
    // .andThen(PathBuilder.createPath(FieldConstants.Trench.left_trench_alliance_preentrance))
    //         .andThen(() -> PathBuilder.targetRotation(() -> Rotation2d.kZero))
    //         .andThen(
    //             () ->
    // PathBuilder.createPath(FieldConstants.Trench.left_trench_alliance_entrance))
    //         .andThen(
    //             PathBuilder.createPath(
    //                 new Pose2d(
    //                     FieldConstants.Trench.left_trench_neutral_entrance, new Rotation2d(0))))
    //         .andThen(PathBuilder.createPath(FieldConstants.FuelField.right_midline_corner)));

    // autoChooser.addOption(
    //     "Sigma",
    //     PathBuilder.pathBuilder.build(
    //         new Path(
    //             new Path.Waypoint(
    //                 FieldConstants.Trench.left_trench_alliance_preentrance, Rotation2d.kZero),
    //             new Path.Waypoint(
    //                 FieldConstants.Trench.left_trench_alliance_entrance, Rotation2d.kZero))));

    // .andThen(
    //     PathBuilder.mergeToKnownPath(
    //         new PathPlannerPath(
    //             FieldConstants.Tower.left_approach,
    //             PathBuilder.getConstraints(),
    //             null,
    //             new GoalEndState(0.0, Rotation2d.k180deg)))));

    autoChooser.addOption(
        "i am right",
        PathBuilder.followNPGPathAccurate(
                NodePathGenerator.generatePath(
                    drive.getPose(),
                    // FieldConstants.Trench.right_trench_neutral_entrance,
                    FieldConstants.Trench.right_trench_neutral_approach,
                    FieldConstants.FuelField.right_close_corner,
                    FieldConstants.FuelField.middle_close_line,
                    FieldConstants.FuelField.right_close_corner,
                    FieldConstants.Trench.right_trench_neutral_preentrance,
                    // FieldConstants.Trench.right_trench_alliance_entrance,
                    // FieldConstants.Trench.right_trench_neutral_preentrance,
                    // FieldConstants.FuelField.right_close_corner,
                    // FieldConstants.FuelField.middle_close_line,
                    // FieldConstants.FuelField.right_close_corner,
                    // FieldConstants.Trench.right_trench_neutral_preentrance,
                    FieldConstants.Trench.right_trench_alliance_entrance,
                    FieldConstants.Tower.right_approach_pos))
            .andThen(PathBuilder.followNPGPathAccurate(FieldConstants.Tower.right_approach)));

    autoChooser.addOption(
        "i am left",
        PathBuilder.followNPGPathAccurate(
                NodePathGenerator.generatePath(
                    drive.getPose(),
                    FieldConstants.Trench.left_trench_neutral_approach,
                    FieldConstants.FuelField.left_close_corner,
                    FieldConstants.FuelField.middle_close_line,
                    FieldConstants.FuelField.left_close_corner,
                    FieldConstants.Trench.left_trench_neutral_preentrance,
                    FieldConstants.Trench.left_trench_alliance_entrance,
                    FieldConstants.Tower.left_approach_pos))
            // .andThen(() -> PathBuilder.targetRotation(() -> Rotation2d.k180deg))
            .andThen(PathBuilder.followNPGPathAccurate(FieldConstants.Tower.left_approach)));

    // autoChooser.addOption(
    //     "i am right",
    //     Commands.sequence(
    //         PathBuilder.followNPGPathAccurate(
    //             NodePathGenerator.generatePath(
    //                 drive.getPose(),
    //                 FieldConstants.right_alliance_shoot,
    //                 FieldConstants.Trench.right_trench_neutral_approach)),
    //         // PathBuilder.followNPGPathAccurate(
    //         //     NodePathGenerator.generatePath(
    //         //         drive.getPose(),
    //         //         FieldConstants.Trench.right_trench_alliance_preentrance,
    //         //         FieldConstants.Trench.right_trench_neutral_preentrance)),
    //         PathBuilder.followNPGPathAccurate(
    //             NodePathGenerator.generatePath(
    //                 drive.getPose(),
    //                 FieldConstants.FuelField.right_close_corner,
    //                 FieldConstants.FuelField.middle_close_line)),
    //         PathBuilder.followNPGPathAccurate(
    //             NodePathGenerator.generatePath(
    //                 drive.getPose(),
    //                 FieldConstants.FuelField.right_close_corner,
    //                 FieldConstants.Trench.right_trench_neutral_preentrance)),
    //         PathBuilder.followNPGPathAccurate(
    //             NodePathGenerator.generatePath(
    //                 drive.getPose(), FieldConstants.Trench.right_trench_alliance_entrance)),
    //         PathBuilder.followNPGPathAccurate(FieldConstants.Tower.right_approach)));

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

  // placeholders
  //   private Command intakeCommand() {
  //     return Commands.none();
  //   }

  //   private Command intakeDown() {
  //     return wrist.down();
  //   }

  //   private Command intakeUp() {
  //     return wrist.stow();
  //   }

  //   private Command shootCommand() {
  //     return Commands.none();
  //   }

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
                        * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
                () ->
                    -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                        * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
                () ->
                    -pilot.getCorrectedRight(Scale.SQUARED).getX()
                        * (pilot.b().getAsBoolean() ? 0.5 : 1.0)))
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
        .rightBumper()
        .whileTrue(
            ScoringCommands.aim(
                    drive,
                    shooter,
                    hood,
                    () ->
                        -pilot.getCorrectedLeft(Scale.SQUARED).getY()
                            * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
                    () ->
                        -pilot.getCorrectedLeft(Scale.SQUARED).getX()
                            * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
                    () -> Constants.ShooterConstants.kMiddleVel)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .beforeStarting(drive.disableVision()))
        .onFalse(drive.enableVision());

    pilot.rightTrigger().onTrue(hopper.runVolts(() -> 0.5, () -> 0.5));
    pilot.leftTrigger().onTrue(intake.intake(() -> 0.7));

    pilot
        .leftBumper()
        .onTrue(intake.intake(() -> -0.7).alongWith(hopper.runVolts(() -> -1.0, () -> -1.0)));

    // pilot.x().onTrue(climber.raise());
    // pilot.y().onTrue(climber.lower());

    pilot.y().toggleOnTrue(climber.raise()).toggleOnFalse(climber.lower());
    pilot.x().onTrue(ScoringCommands.goToClimb(drive, climber));

    // copilot.a().onTrue(wrist.down());
    // copilot.x().onTrue(wrist.stow());

    copilot.a().toggleOnTrue(wrist.down()).toggleOnFalse(wrist.stow());
    copilot.x().onTrue(ScoringCommands.shake(wrist));

    copilot.b().whileTrue(climber.runVolts(() -> -copilot.getLeftY(Scale.LINEAR)));
    copilot.y().whileTrue(wrist.runWrist(() -> -copilot.getLeftY(Scale.LINEAR)));

    copilot.getUpButton().onTrue(drive.enableVision());
    copilot.getDownButton().onTrue(drive.disableVision());
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
        drive.setPose(new Pose2d(new Translation2d(3.57, 2), Rotation2d.kZero));
      } else if (DriverStation.getLocation().getAsInt() == 2) {
        drive.setPose(
            new Pose2d(
                new Translation2d(3.57, Units.inchesToMeters(317.69) / 2), Rotation2d.kZero));
      } else if (DriverStation.getLocation().getAsInt() == 1) {
        drive.setPose(
            new Pose2d(
                new Translation2d(3.57, Units.inchesToMeters(317.69) - 2), Rotation2d.kZero));
      }
    } else if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      if (DriverStation.getLocation().getAsInt() == 3) {
        drive.setPose(AllianceFlip.apply(new Pose2d(new Translation2d(3.57, 2), Rotation2d.kZero)));
      } else if (DriverStation.getLocation().getAsInt() == 2) {
        drive.setPose(
            AllianceFlip.apply(
                new Pose2d(
                    new Translation2d(3.57, Units.inchesToMeters(317.69) / 2), Rotation2d.kZero)));
      } else if (DriverStation.getLocation().getAsInt() == 1) {
        drive.setPose(
            AllianceFlip.apply(
                new Pose2d(
                    new Translation2d(3.57, Units.inchesToMeters(317.69) - 2), Rotation2d.kZero)));
      }
    }
  }

  public void periodic() {

    Logger.recordOutput("Drive/Angle At Setpoint?", Constants.DriveConstants.ANGLE_PID.atGoal());

    // Logger.recordOutput("State/Robot Mode", Constants.Robot.robotMode);

    // testing placeholder
    // if (AllianceFlip.flipX(drive.getPose().getX())
    //     < FieldConstants.alliance_zone_x - Constants.Robot.B_LENGTH) {
    //   Constants.Robot.robotMode = Constants.RobotMode.SHOOT;
    // } else {
    //   Constants.Robot.robotMode = Constants.RobotMode.NONE;
    // }
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.Robot.currentMode != Constants.Mode.SIM) return;

    simvis.update();

    Logger.recordOutput("FieldSimulation/RobotPosition", drive.getPose());
  }
}
