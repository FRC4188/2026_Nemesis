// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.pathbuilder.*;
import frc.robot.CSPLib.csppathing.CSPPathing;
import frc.robot.CSPLib.csppathing.PosePathing;
// import frc.robot.CSPLib.csppathing.PathBuilder;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.commands.Scoring.AutoCommands;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.commands.drive.DriveCommands;
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
import java.util.Random;
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
  private final Vision vis;
  private final SimulationVisualizer simvis;

  // Controller
  private final CSP_Controller pilot = new CSP_Controller(Constants.Controller.kPilotPort);
  private final CSP_Controller copilot = new CSP_Controller(Constants.Controller.kCopilotPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<AutoCommands.Start> startChooser;
  private final LoggedDashboardChooser<AutoCommands.Swipe> swipeChooser;
  private final LoggedDashboardChooser<AutoCommands.Cycle> cycleChooser;

  // private final CSPPilot csppilot =
  //     new CSPPilot(
  //         new CSPPilot.CSPProfile(
  //                 new CSPPilot.CSPConstraints()
  //                     .withVelocity(Constants.DriveConstants.DRIVE_MAXVEL)
  //                     .withAcceleration(Constants.DriveConstants.DRIVE_MAXACC * 1.3)
  //                     .withJerk(10))
  //             .withErrorXY(edu.wpi.first.units.Units.Centimeters.of(2))
  //             .withErrorTheta(edu.wpi.first.units.Units.Degrees.of(0.5))
  //             .withBeelineRadius(edu.wpi.first.units.Units.Centimeters.of(5)));

  public RobotContainer() {
    drive = Drive.getInstance();
    vis = Vision.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    hopper = Hopper.getInstance();
    intake = Intake.getInstance();
    wrist = Wrist.getInstance();
    simvis =
        new SimulationVisualizer(
            "Models",
            () -> Units.degreesToRadians(wrist.getAngle()),
            () -> Units.degreesToRadians(hood.getAngle()));

    // Set up auto routines
    // PathBuilder.configure(drive); // Add all subsystems as parameters later
    PathBuilder.configureField(FieldConstants.field_width, FieldConstants.field_length);
    PathBuilder.configureDrive(
        true,
        2,
        Constants.DriveConstants.ANGLE_TOL,
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        drive::stop,
        drive::runVelocity,
        Constants.DriveConstants.DRIVE_PID,
        Constants.DriveConstants.ANGLE_PID,
        Constants.DriveConstants.PP_CONFIG,
        new PathConstraints(
            Constants.DriveConstants.DRIVE_MAXVEL,
            Constants.DriveConstants.DRIVE_MAXACC,
            Constants.DriveConstants.ANGLE_MAXVEL * 0.8,
            Constants.DriveConstants.ANGLE_MAXACC * 0.8),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drive);
    // PBExperimental.configure(drive);

    // These 4 choosers are subbing for PB's dashboard fyi
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    startChooser = new LoggedDashboardChooser<>("Start Choices");
    startChooser.addOption("Left Trench", AutoCommands.Start.LEFT);
    startChooser.addDefaultOption("Right Trench", AutoCommands.Start.RIGHT);
    swipeChooser = new LoggedDashboardChooser<>("Swipe Choices");
    swipeChooser.addDefaultOption("Center Swipe", AutoCommands.Swipe.CENTER);
    swipeChooser.addOption("Close Swipe", AutoCommands.Swipe.CLOSE);
    cycleChooser = new LoggedDashboardChooser<>("Cycle Choices");
    cycleChooser.addDefaultOption("2", AutoCommands.Cycle.DOUBLE);
    cycleChooser.addOption("None", AutoCommands.Cycle.NONE);
    cycleChooser.addOption("1.5", AutoCommands.Cycle.NZ);

    autoChooser.addOption(
        "PsuedoBoard Delayed",
        Commands.defer(
            () ->
                AutoCommands.pseudoBoard(
                    startChooser.get(), swipeChooser.get(), cycleChooser.get()),
            Set.of(shooter, hopper, wrist, intake, hood)));
    autoChooser.addDefaultOption("PsuedoBoard", AutoCommands.constructedAuto);

    autoChooser.addOption(
        "FORWARD Right Disruptor",
        AutoCommands.rightDisrupt(drive, intake, hopper, shooter, wrist, hood));

    autoChooser.addOption(
        "FORWARD Left Disruptor",
        AutoCommands.leftDisrupt(drive, intake, hopper, shooter, wrist, hood));

    autoChooser.addOption("BACKWARD Full Depot", AutoCommands.fullDepot());

    autoChooser.addOption("Nothing", Commands.none());

    PathPlannerPath path =
        PathBuilder.build(
            new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.Trench.right_trench_center.plus(new Translation2d(0, -0.18)),
                        Rotation2d.kCCW_90deg))
                .withStartingSpeed(5)
                .withStartingRotation(Rotation2d.kCCW_90deg)
                .withOverrideRotations(
                    new RotationTarget(0.97, Rotation2d.fromDegrees(87.075)),
                    new RotationTarget(0.60, Rotation2d.kCCW_90deg),
                    new RotationTarget(2.00, Rotation2d.fromDegrees(110.726)),
                    new RotationTarget(3.00, Rotation2d.fromDegrees(-95.856)),
                    new RotationTarget(3.34, Rotation2d.fromDegrees(-85.402)))
                .withHeading(Rotation2d.fromDegrees(61.763))
                .withControlDistances(0, 0.250),
            new PathBuilder.Target(new Pose2d(7.355, 1.523 - 0.18, Rotation2d.kZero))
                .withHeading(Rotation2d.fromDegrees(66.360))
                .withControlDistances(1.517, 0.476),
            new PathBuilder.Target(new Pose2d(7.614, 3.051, Rotation2d.kZero))
                .withHeading(Rotation2d.fromDegrees(120.689))
                .withControlDistances(0.288, 1.250),
            new PathBuilder.Target(new Pose2d(5.968, 3.051, Rotation2d.kZero))
                .withHeading(Rotation2d.fromDegrees(-104.349))
                .withControlDistances(0.955, 0.310),
            new PathBuilder.Target(new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
                .withHeading(Rotation2d.fromDegrees(99.792))
                .withControlDistances(0.250, 0)
                .withEndingRotation(Rotation2d.kZero)
                .withEndingSpeed(2));

    PathPlannerPath path1 =
        PathBuilder.build(
            new PathBuilder.Target(new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
                .withStartingSpeed(2),
            new PathBuilder.Target(
                new Pose2d(
                    FieldConstants.Trench.right_trench_center.plus(new Translation2d(0, -0.15)),
                    Rotation2d.kZero)));

    // autoChooser.addOption(
    //     "test csppilot",
    //     new CSPPathing(csppilot, Constants.DriveConstants.ANGLE_PID)
    //         .withStartPose(
    //             new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
    //         .addPaths(path, path1)
    //         .build());

    autoChooser.addOption(
        "pathhhhh",
        new CSPPathing(Constants.DriveConstants.PILOT, Constants.DriveConstants.ANGLE_PID)
            .withStartPose(
                new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
            .addPath(path)
            .addPath(path1)
            .build());

    autoChooser.addOption(
        "AP Testing",
        new PosePathing(Constants.DriveConstants.ANGLE_PID)
            .withStartPose(
                new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
            .withHandoffThreshold(0.5)
            .addWaypoint(
                new Pose2d(
                    FieldConstants.Trench.right_trench_neutral_preentrance, Rotation2d.kCCW_90deg))
            .addWaypoint(
                new Pose2d(
                    FieldConstants.FuelField.right_midline_corner, Rotation2d.fromDegrees(115)))
            .addWaypoint(new Pose2d(FieldConstants.field_center, Rotation2d.fromDegrees(115)))
            .build());

    autoChooser.addOption(
        "nz bump",
        Commands.sequence(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))),
            PathBuilder.path(
                new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
                    .withStartingSpeed(5),
                new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.Trench.right_trench_neutral_preentrance,
                        Rotation2d.kCCW_90deg)),
                new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.FuelField.intake_right_midline_corner,
                        Rotation2d.fromDegrees(110))),
                new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.FuelField.intake_midline, Rotation2d.fromDegrees(110)))),
            PathBuilder.path(
                new PathBuilder.Target(
                    new Pose2d(FieldConstants.field_center, Rotation2d.fromDegrees(110))),
                new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.FuelField.middle_close_line.plus(new Translation2d(0, -0.5)),
                        Rotation2d.kZero)),
                new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.Bump.right_bump_neutral_entrance.plus(
                            new Translation2d(0.5, 0)),
                        Rotation2d.kZero)),
                new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Bump.right_bump_alliance_entrance.plus(
                                new Translation2d(-1, 0)),
                            Rotation2d.kZero))
                    .withEndingSpeed(2))));

    PathPlannerPath cpath =
        PathBuilder.build(
            new PathBuilder.Target(
                    new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
                .withStartingSpeed(5),
            new PathBuilder.Target(
                new Pose2d(
                    FieldConstants.Trench.right_trench_neutral_preentrance, Rotation2d.kCCW_90deg)),
            new PathBuilder.Target(
                new Pose2d(
                    FieldConstants.FuelField.intake_right_midline_corner,
                    Rotation2d.fromDegrees(110))),
            new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.FuelField.intake_midline, Rotation2d.fromDegrees(110)))
                .withEndingSpeed(2));

    // autoChooser.addOption(
    //     "nz bump pose",
    //     new CSPPathing(csppilot, Constants.DriveConstants.ANGLE_PID)
    //         .withStartPose(
    //             new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg))
    //         .addPath(cpath)
    //         .build());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization());
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization());

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
                    || pilot.rightBumper().getAsBoolean()
                    || pilot.a().getAsBoolean()
                    || pilot.x().getAsBoolean()));

    driveInput.whileTrue(
        DriveCommands.joystickCombined(
            () -> -pilot.getCorrectedLeft(Scale.SQUARED).getY(),
            //  * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () -> -pilot.getCorrectedLeft(Scale.SQUARED).getX(),
            // * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () -> -pilot.getCorrectedRight(Scale.SQUARED).getX(),
            //     * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () ->
                (pilot.x().getAsBoolean())
                    ? (drive.getPose().getTranslation().getY() > FieldConstants.field_center.getY())
                        ? Rotation2d.kCW_90deg
                        : Rotation2d.kCCW_90deg
                    : (pilot.a().getAsBoolean()
                        ? drive
                            .getPose()
                            .getTranslation()
                            .nearest(
                                List.of(
                                    AllianceFlip.apply(FieldConstants.Depot.left_far_corner),
                                    AllianceFlip.apply(
                                        new Translation2d(
                                            FieldConstants.Depot.left_far_corner.getX(),
                                            FieldConstants.field_width
                                                - FieldConstants.Depot.left_far_corner.getY()))))
                            .minus(drive.getPose().getTranslation())
                            .getAngle()
                        : AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                            .minus(drive.getPose().getTranslation())
                            .getAngle()),
            () ->
                pilot.rightBumper().getAsBoolean()
                    || pilot.a().getAsBoolean()
                    || pilot.x().getAsBoolean()));

    pilot.rightBumper().whileTrue(ScoringCommands.staticAim());

    pilot.a().whileTrue(ScoringCommands.passAim());
    pilot
        .y()
        .or(() -> pilot.b().getAsBoolean())
        .whileTrue(ScoringCommands.manualAim(() -> (pilot.b().getAsBoolean()) ? 3.5 : 12));

    pilot
        .getRightTButton()
        .whileTrue(
            Commands.either(
                ScoringCommands.passShoot(),
                Commands.either(
                    // ScoringCommands.dataShoot(),
                    ScoringCommands.staticShoot(),
                    ScoringCommands.manualShoot(() -> (pilot.b().getAsBoolean()) ? 3.5 : 12),
                    () -> pilot.rightBumper().getAsBoolean()),
                () -> pilot.a().getAsBoolean()));

    // pilot.getRightTButton().whileTrue(new WaitCommand(4).andThen(ScoringCommands.testShake()));
    pilot
        .getRightTButton()
        .whileTrue(ScoringCommands.slowUp())
        .onFalse(ScoringCommands.downNoStall());

    pilot
        .getLeftTButton()
        .whileTrue(Commands.runEnd(() -> intake.intakeVolts(8.75), intake::stop, intake));

    pilot
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runEnd(() -> hopper.runHopper(-6.0, 0), hopper::stop, hopper),
                Commands.runEnd(() -> intake.ejectVolts(6.0), intake::stop, intake)));

    // Commands.either(
    //     ScoringCommands.halfShake(), ScoringCommands.fullShake(), () ->
    // copilot.b().getAsBoolean());

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
            AllianceFlip.apply(
                new Pose2d(
                    FieldConstants.Trench.left_trench_alliance_entrance.getX()
                        + Constants.Robot.B_LENGTH / 2,
                    FieldConstants.Trench.left_trench_alliance_entrance.getY(),
                    Rotation2d.kCW_90deg)));
        // new Pose2d(0, 0, Rotation2d.kZero));
      }
    }

    // drive.setPose(new Pose2d(FieldConstants.FuelField.second_intake_right_close_corner,
    // Rotation2d.kCW_90deg));
  }

  char autoWinner = ' '; // this is so stupid

  public void preperiodic() {

    if (startChooser.get() != AutoCommands.curStart
        || cycleChooser.get() != AutoCommands.curCycle
        || swipeChooser.get() != AutoCommands.curSwipe) {
      AutoCommands.constructedAuto =
          AutoCommands.pseudoBoard(startChooser.get(), swipeChooser.get(), cycleChooser.get());
      AutoCommands.curStart = startChooser.get();
      AutoCommands.curCycle = cycleChooser.get();
      AutoCommands.curSwipe = swipeChooser.get();
    }

    autoChooser.addDefaultOption("PsuedoBoard", AutoCommands.constructedAuto);
  }

  public void periodic() {

    // shooter.setVelocityRPM(shooterRPMset.getAsDouble());

    // Logger.recordOutput("Drive/Angle", drive.getPose().getRotation().getDegrees());
    // Logger.recordOutput(
    //     "Drive/Setpoint", Constants.DriveConstants.ANGLE_PID.getSetpoint().position);
    Logger.recordOutput("Drive/At Goal?", Constants.DriveConstants.ANGLE_PID.atGoal());

    Logger.recordOutput(
        "Drive/Distance From Hub",
        AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
            .getDistance(drive.getPose().getTranslation()));

    boolean hubState = false;
    double timeLeftInShift = 0.0;

    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getGameSpecificMessage().length() > 0) {
        autoWinner = DriverStation.getGameSpecificMessage().charAt(0);
      }
    } else if (DriverStation.isDSAttached()) {
      if (autoWinner == ' ') {
        autoWinner = new Random().nextBoolean() ? 'B' : 'R'; // for practice
      }
    } else {
      autoWinner = 'B'; // fallback
    }

    if (DriverStation.isTeleop()) {
      Logger.recordOutput(
          "Won Auto?",
          (autoWinner == 'B' && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
              || (autoWinner == 'R'
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red));
      if (DriverStation.getMatchTime() <= 140 && DriverStation.getMatchTime() > 131) {
        hubState = true;
        timeLeftInShift = DriverStation.getMatchTime() - 130;
      } else if (DriverStation.getMatchTime() <= 131 && DriverStation.getMatchTime() > 106) {
        if ((autoWinner == 'B' && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            || (autoWinner == 'R'
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
          hubState = false;
        } else {
          hubState = true;
        }
        timeLeftInShift = DriverStation.getMatchTime() - 105;
      } else if (DriverStation.getMatchTime() <= 106 && DriverStation.getMatchTime() > 81) {
        if ((autoWinner == 'B' && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            || (autoWinner == 'R'
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
          hubState = true;
        } else {
          hubState = false;
        }
        timeLeftInShift = DriverStation.getMatchTime() - 80;
      } else if (DriverStation.getMatchTime() <= 81 && DriverStation.getMatchTime() > 56) {
        if ((autoWinner == 'B' && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            || (autoWinner == 'R'
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
          hubState = false;
        } else {
          hubState = true;
        }
        timeLeftInShift = DriverStation.getMatchTime() - 55;
      } else if (DriverStation.getMatchTime() <= 56 && DriverStation.getMatchTime() > 31) {
        if ((autoWinner == 'B' && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            || (autoWinner == 'R'
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) {
          hubState = true;
        } else {
          hubState = false;
        }
        timeLeftInShift = DriverStation.getMatchTime() - 30;
      } else if (DriverStation.getMatchTime() <= 31) {
        hubState = true;
        timeLeftInShift = DriverStation.getMatchTime();
      }
    } else {
      hubState = true; // we're in auto
      timeLeftInShift = DriverStation.getMatchTime();
      Logger.recordOutput("Won Auto?", false);
    }
    Logger.recordOutput("Is Our Shift?", hubState);
    Logger.recordOutput("Time Left In Shift", Math.floor(timeLeftInShift));
  }

  // so weird
  public void genericReset() {
    autoWinner = ' ';
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.Robot.currentMode != Constants.Mode.SIM) return;

    simvis.update();

    Logger.recordOutput("FieldSimulation/RobotPosition", drive.getPose());
  }
}
