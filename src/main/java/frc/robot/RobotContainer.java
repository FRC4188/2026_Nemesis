// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.team4188.voyager.VoyagerLib;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.pathbuilder.*;
// import frc.robot.CSPLib.csppathing.PathBuilder;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.commands.SOTM;
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
  // private final LoggedDashboardChooser<AutoCommands.Start> startChooser;
  // private final LoggedDashboardChooser<AutoCommands.Swipe> swipeChooser;
  // private final LoggedDashboardChooser<AutoCommands.Cycle> cycleChooser;

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

    VoyagerLib.configure(
        drive,
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        (ChassisSpeeds commanded) -> {
          double omega;
          if (SOTM.autonLocked) {
            omega =
                Constants.DriveConstants.ANGLE_PID.calculate(
                    drive.getRotation().getRadians(),
                    AllianceFlip.apply(
                            SOTM.lookahead(
                                    new Pose2d(FieldConstants.Hub.hub_center_2d, new Rotation2d()),
                                    drive.getChassisSpeeds(),
                                    commanded,
                                    SOTM.TOF_SECONDS)
                                .getTranslation())
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .minus(Constants.DriveConstants.local_offset)
                        .getRadians());
          } else {
            omega = commanded.omegaRadiansPerSecond;
          }

          drive.runVelocity(
              new ChassisSpeeds(
                  commanded.vxMetersPerSecond * ((SOTM.autonLocked) ? 0.17 : 1.0),
                  commanded.vyMetersPerSecond * ((SOTM.autonLocked) ? 0.17 : 1.0),
                  omega));
        },
        true);
    VoyagerLib.setDefaultGlobalConstraints(
        4.5 / 3, // maxVelocityMetersPerSec
        10.0, // maxAccelerationMetersPerSec2
        Math.toDegrees(Constants.DriveConstants.ANGLE_MAXVEL), // maxVelocityDegPerSec
        Math.toDegrees(Constants.DriveConstants.ANGLE_MAXACC), // maxAccelerationDegPerSec2
        0.03, // endTranslationToleranceMeters
        2.0, // endRotationToleranceDeg
        0.2 // intermediateHandoffRadiusMeters
        );
    VoyagerLib.setModuleOrientationConsumer(drive::setModuleOrientations);
    VoyagerLib.setPIDControllers(
        new PIDController(5, 0, 0.4), new PIDController(5, 0, 0.4), new PIDController(2, 0, 0));
    System.out.println(Drive.DRIVE_BASE_RADIUS);

    VoyagerLib.addEvent(
        "ShootOnTheMove",
        SOTM.dynamicShoot(
                () -> new ChassisSpeeds(0, 0, 0),
                drive::getChassisSpeeds,
                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d))
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    VoyagerLib.addEvent(
        "EndSOTM",
        Commands.runOnce(() -> SOTM.autonLocked = false)
            .andThen(hood.idle())
            .andThen(shooter.idle())
            .andThen(hopper.idle())
            .andThen(intake.idle()));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

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
                    || pilot.getRightTButton().getAsBoolean()
                    || pilot.rightBumper().getAsBoolean()
                    || pilot.y().getAsBoolean()
                    || pilot.b().getAsBoolean()));

    driveInput.whileTrue(
        DriveCommands.joystickCombined(
            () -> -pilot.getCorrectedLeft(Scale.SQUARED).getY(),
            //  * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () -> -pilot.getCorrectedLeft(Scale.SQUARED).getX(),
            // * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () -> -pilot.getCorrectedRight(Scale.WILL).getX(),
            //     * (pilot.b().getAsBoolean() ? 0.5 : 1.0),
            () ->
                ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                            && drive.getPose().getX()
                                <= AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX())
                        || (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                            && drive.getPose().getX()
                                >= AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX()))
                    ? AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                    : drive
                        .getPose()
                        .getTranslation()
                        .nearest(
                            List.of(
                                AllianceFlip.apply(FieldConstants.Depot.left_far_corner),
                                AllianceFlip.flipY(
                                    AllianceFlip.apply(FieldConstants.Depot.left_far_corner))))
                        .minus(drive.getPose().getTranslation())
                        .getAngle(),
            () -> pilot.getRightTButton().getAsBoolean()));

    pilot.getRightTButton().whileTrue(ScoringCommands.shoot(() -> 0, pilot.getLeftTButton()));

    pilot
        .y()
        .or(() -> pilot.b().getAsBoolean())
        .whileTrue(
            ScoringCommands.shoot(
                () -> (pilot.b().getAsBoolean()) ? 3.5 : 12, pilot.getLeftTButton()));

    pilot
        .getLeftTButton()
        .whileTrue(Commands.runEnd(() -> intake.intakeVolts(8.75), intake::stop, intake));

    pilot
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                Commands.runEnd(() -> hopper.runHopper(-6.0, 0), hopper::stop, hopper),
                Commands.runEnd(() -> intake.ejectVolts(6.0), intake::stop, intake)));

    copilot
        .y()
        .whileTrue(
            Commands.runEnd(
                () -> wrist.runWristVolts(3 * -copilot.getLeftY(Scale.LINEAR)),
                wrist::stop,
                wrist));

    copilot.a().onTrue(ScoringCommands.downNoStall());
    copilot.x().onTrue(ScoringCommands.goodStow());
    copilot.rightBumper().onTrue(ScoringCommands.forceDown());

    copilot.b().whileTrue(ScoringCommands.lowerIntakeTorque());

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

    copilot.povLeft().onTrue(ScoringCommands.shooterIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return VoyagerLib.runSelectedAuto();
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

  char autoWinner = ' '; // this is so stupid xd

  public void preperiodic() {

    // if (startChooser.get() != AutoCommands.curStart
    //     || cycleChooser.get() != AutoCommands.curCycle
    //     || swipeChooser.get() != AutoCommands.curSwipe) {
    //   //   AutoCommands.constructedAuto =
    //   //       AutoCommands.pseudoBoard(startChooser.get(), swipeChooser.get(),
    // cycleChooser.get());
    //   AutoCommands.custom = AutoCommands.newAuto(startChooser.get(), cycleChooser.get());
    //   AutoCommands.curStart = startChooser.get();
    //   AutoCommands.curCycle = cycleChooser.get();
    //   AutoCommands.curSwipe = swipeChooser.get();
    // }

    // autoChooser.addDefaultOption("PseudoBoard", AutoCommands.constructedAuto);
    // autoChooser.addDefaultOption("NewBoard", AutoCommands.custom);
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
