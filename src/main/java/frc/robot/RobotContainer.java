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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.CSPLib.ppp.PathBuilder;
import frc.robot.CSPLib.ppp.PathBuilder.Target.RotationMode;
import frc.robot.commands.Scoring.AutoCommands;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.simulation.SimulationVisualizer;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.Set;
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
  private final SimulationVisualizer simvis;

  // Controller
  private final CSP_Controller pilot = new CSP_Controller(Constants.Controller.kPilotPort);
  private final CSP_Controller copilot = new CSP_Controller(Constants.Controller.kCopilotPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<AutoCommands.Start> startChooser;
  private final LoggedDashboardChooser<AutoCommands.Swipe> swipeChooser;
  private final LoggedDashboardChooser<AutoCommands.Climb> climbChooser;

  public RobotContainer() {
    drive = Drive.getInstance();
    vis = Vision.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    hopper = Hopper.getInstance();
    intake = Intake.getInstance();
    wrist = Wrist.getInstance();
    climber = Climber.getInstance();
    simvis = new SimulationVisualizer(
                "Models",
                () -> Units.degreesToRadians(wrist.getAngle()),
                () -> Units.degreesToRadians(hood.getAngle()),
                () -> Units.inchesToMeters(climber.getHeight()));

    // Set up auto routines
    // PathBuilder.configure(drive); // Add all subsystems as parameters later
    PathBuilder.configureDrive(
        true,
        2,
        Constants.DriveConstants.ANGLE_TOL,
        () -> drive.getPose(),
        drive::setPose,
        () -> drive.getChassisSpeeds(),
        () -> drive.stop(),
        drive::runVelocity,
        Constants.DriveConstants.DRIVE_PID,
        Constants.DriveConstants.ANGLE_PID,
        Constants.DriveConstants.PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);
    // PBExperimental.configure(drive);

    // These 4 choosers are subbing for PB's dashboard fyi
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    startChooser = new LoggedDashboardChooser<>("Start Choices");
    startChooser.addOption("Left Trench", AutoCommands.Start.LEFT);
    startChooser.addOption("Right Trench", AutoCommands.Start.RIGHT);
    swipeChooser = new LoggedDashboardChooser<>("Swipe Choices");
    swipeChooser.addOption("Center Swipe", AutoCommands.Swipe.CENTER);
    swipeChooser.addOption("Close Swipe", AutoCommands.Swipe.CLOSE);
    climbChooser = new LoggedDashboardChooser<>("Climb Choices");
    climbChooser.addOption("Climb", AutoCommands.Climb.CLIMB);
    climbChooser.addOption("No Climb", AutoCommands.Climb.NONE);
    climbChooser.addOption("1.5 Neutral", AutoCommands.Climb.NZ);
    climbChooser.addOption("2nd Swipe", AutoCommands.Climb.DOUBLE);

    autoChooser.addOption(
        "PsuedoBoard",
        Commands.defer(
            () ->
                AutoCommands.pseudoBoard(
                    startChooser.get(),
                    swipeChooser.get(),
                    climbChooser.get(),
                    drive,
                    shooter,
                    hood,
                    hopper,
                    intake,
                    wrist,
                    climber),
            Set.of(drive, shooter, hood, hopper, intake, wrist, climber)));

    // autoChooser.addOption(
    //     "testing drive idk",
    //     Commands.runOnce(() -> drive.setPose(new Pose2d()));
    //     );

    autoChooser.addOption(
        "FORWARD Right Disruptor",
        AutoCommands.rightDisrupt(drive, intake, hopper, shooter, wrist, hood));

    autoChooser.addOption(
        "FORWARD Left Disruptor",
        AutoCommands.leftDisrupt(drive, intake, hopper, shooter, wrist, hood));

    autoChooser.addOption("Climb Only Right", AutoCommands.climbRight(climber));
    autoChooser.addOption("Climb Only Left", AutoCommands.climbLeft(climber));
    autoChooser.addOption("Nothing", Commands.none());

    autoChooser.addOption("pathbuilder test 2", testCirclePath().repeatedly());

    autoChooser.addOption(
        "pathbuilder test 3",
        Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kZero)))
            .andThen(
                Commands.sequence(
                    PathBuilder.path(
                        new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_center, Rotation2d.kZero)),
                        new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                Rotation2d.kZero)),
                        new PathBuilder.Target(new Pose2d(7.216, 2.050, Rotation2d.kZero))
                            .withRotationMode(RotationMode.FOLLOW),
                        new PathBuilder.Target(new Pose2d(7.033, 3.040, Rotation2d.kZero))
                            .withRotationMode(RotationMode.FOLLOW),
                        new PathBuilder.Target(new Pose2d(6.172, 3.276, Rotation2d.kZero))
                            .withRotationMode(RotationMode.FOLLOW),
                        new PathBuilder.Target(new Pose2d(6.000, 1.921, Rotation2d.kZero))
                            .withRotationMode(RotationMode.FOLLOW))),
                PathBuilder.path(
                    new PathBuilder.Target(new Pose2d(6.000, 1.921, Rotation2d.kZero)).withCurve(1),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_neutral_preentrance,
                            Rotation2d.kZero)))));

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  public static Command testCirclePath() {
    double cx = 5.0;
    double cy = 5.0;
    double r = 1.0;

    return PathBuilder.path(
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx + r, cy), Rotation2d.fromDegrees(90)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx + 0.707, cy + 0.707), Rotation2d.fromDegrees(135)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx, cy + r), Rotation2d.fromDegrees(180)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx - 0.707, cy + 0.707), Rotation2d.fromDegrees(225)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx - r, cy), Rotation2d.fromDegrees(270)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx - 0.707, cy - 0.707), Rotation2d.fromDegrees(315)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(new Pose2d(new Translation2d(cx, cy - r), Rotation2d.fromDegrees(0)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW),
        new PathBuilder.Target(
                new Pose2d(new Translation2d(cx + 0.707, cy - 0.707), Rotation2d.fromDegrees(45)))
            .withRotationMode(PathBuilder.Target.RotationMode.FOLLOW));
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
                    || pilot.rightBumper().getAsBoolean()
                    || pilot.a().getAsBoolean()));

    driveInput.whileTrue(
        DriveCommands.joystickCombined(
            () -> -pilot.getCorrectedLeft(Scale.SQUARED).getY(),
            //  * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () -> -pilot.getCorrectedLeft(Scale.SQUARED).getX(),
            // * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () -> -pilot.getCorrectedRight(Scale.SQUARED).getX(),
            //     * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
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
                    : AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getAngle()),
            () -> pilot.rightBumper().getAsBoolean() || pilot.a().getAsBoolean()));

    pilot.rightBumper().whileTrue(ScoringCommands.staticAim());
    pilot.a().whileTrue(ScoringCommands.passAim());
    pilot
        .x()
        .or(() -> pilot.b().getAsBoolean())
        .whileTrue(ScoringCommands.manualAim(() -> (pilot.b().getAsBoolean()) ? 3.5 : 12));

    pilot
        .getRightTButton()
        .whileTrue(
            Commands.either(
                ScoringCommands.passShoot(),
                Commands.either(
                    // ScoringCommands.dataShoot(shooter, hopper),
                    ScoringCommands.staticShoot(),
                    ScoringCommands.manualShoot(
                        () -> (pilot.b().getAsBoolean()) ? 3.5 : 12),
                    () -> pilot.rightBumper().getAsBoolean()),
                () -> pilot.a().getAsBoolean()));

    pilot.getRightTButton().whileTrue(ScoringCommands.shake());

    pilot
        .getLeftTButton()
        .whileTrue(Commands.runEnd(() -> intake.intakeVolts(7.25), intake::stop, intake));

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
                () -> climber.runClimberVolts(8 * -copilot.getCorrectedLeft(Scale.LINEAR).getY()),
                climber::stop,
                climber));
    copilot
        .y()
        .whileTrue(
            Commands.runEnd(
                () -> wrist.runWristVolts(5 * -copilot.getLeftY(Scale.LINEAR)),
                wrist::stop,
                wrist));

    copilot.x().onTrue(ScoringCommands.downNoStall());
    copilot.a().onTrue(ScoringCommands.goodStow());

    copilot.leftBumper().whileTrue(ScoringCommands.shake());
    copilot.rightBumper().onTrue(ScoringCommands.forceDown());

    copilot
        .getLeftTButton()
        .toggleOnTrue(
            Commands.startEnd(() -> wrist.enableShake(false), () -> wrist.enableShake(true)));

    copilot.povLeft().onTrue(Commands.runOnce(climber::zero));
    copilot.povRight().onTrue(Commands.runOnce(wrist::zero));
    copilot.povUp().onTrue(Commands.runOnce(hood::addOne));
    copilot.povDown().onTrue(Commands.runOnce(hood::subOne));
    copilot
        .getRightTButton()
        .toggleOnTrue(
            Commands.startEnd(() -> vis.enableVision(false), () -> vis.enableVision(true)));
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
            // AllianceFlip.apply(
            //     new Pose2d(
            //         FieldConstants.Trench.left_trench_alliance_entrance.getX()
            //             + Constants.Robot.B_LENGTH / 2,
            //         FieldConstants.Trench.left_trench_alliance_entrance.getY(),
            //         Rotation2d.kCW_90deg)));
            new Pose2d(0, 0, Rotation2d.kZero));
      }
    }

    // drive.setPose(new Pose2d(FieldConstants.FuelField.second_intake_right_close_corner,
    // Rotation2d.kCW_90deg));
  }

  public void periodic() {
    // Logger.recordOutput("Drive/Angle", drive.getPose().getRotation().getDegrees());
    // Logger.recordOutput(
    //     "Drive/Setpoint", Constants.DriveConstants.ANGLE_PID.getSetpoint().position);
    Logger.recordOutput("Drive/At Goal?", Constants.DriveConstants.ANGLE_PID.atGoal());

    // Logger.recordOutput(
    //     "Drive/Distance From Hub",
    //     FieldConstants.Hub.hub_center_2d.getDistance(drive.getPose().getTranslation()));
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.Robot.currentMode != Constants.Mode.SIM) return;

    simvis.update();

    Logger.recordOutput("FieldSimulation/RobotPosition", drive.getPose());
  }
}
