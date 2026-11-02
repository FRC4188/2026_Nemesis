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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathbuilder.*;
// import frc.robot.CSPLib.csppathing.PathBuilder;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.inputs.CSP_Controller.Scale;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.simulation.SimulationVisualizer;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.Random;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Wires controllers and autonomous events to the finite state machine and robot subsystems. */
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
  private final Superstructure superstructure;

  // Controller
  private final CSP_Controller pilot = new CSP_Controller(Constants.Controller.kPilotPort);
  private final CSP_Controller copilot = new CSP_Controller(Constants.Controller.kCopilotPort);

  private boolean previousStart;
  private boolean previousPovUp;
  private boolean previousPovDown;
  private boolean previousPovLeft;
  private boolean previousPovRight;
  private boolean previousCopilotX;
  private boolean previousCopilotRightBumper;
  private boolean previousVisionToggle;
  private boolean previousManualWrist;

  // Dashboard inputs
  private final LoggedDashboardChooser<String> autoChooser;

  // private final FollowPath.Builder pathBuilder;

  public RobotContainer() {
    drive = Drive.getInstance();
    vis = Vision.getInstance();
    hood = Hood.getInstance();
    shooter = Shooter.getInstance();
    hopper = Hopper.getInstance();
    intake = Intake.getInstance();
    wrist = Wrist.getInstance();
    superstructure = new Superstructure(drive, hood, shooter, hopper, intake, wrist, vis);
    wrist.requestHold();
    simvis =
        new SimulationVisualizer(
            "Models",
            () -> Units.degreesToRadians(wrist.getAngle()),
            () -> Units.degreesToRadians(hood.getAngle()));

    // pathBuilder = new FollowPath.Builder(
    //     drive,
    //     drive::getPose,
    //     drive::getChassisSpeeds,
    //     drive::runVelocity,
    //     new PIDController(5.2, 0.0, 0.4),
    //     new PIDController(5, 0.0, 0.4),
    //     new PIDController(1.5, 0.0, 0.0)
    // ).withDefaultShouldFlip()
    // .withTRatioBasedTranslationHandoffs(true);

    VoyagerLib.configure(
        drive, drive::getPose, drive::setPose, drive::getChassisSpeeds, drive::runVelocity, true);
    VoyagerLib.setDefaultGlobalConstraints(
        4.5, // maxVelocityMetersPerSec
        12, // maxAccelerationMetersPerSec2
        720, // maxVelocityDegPerSec
        1500, // maxAccelerationDegPerSec2
        0.03, // endTranslationToleranceMeters
        2.0, // endRotationToleranceDeg
        0.2 // intermediateHandoffRadiusMeters
        );
    VoyagerLib.setModuleOrientationConsumer(drive::setModuleOrientations);
    VoyagerLib.setPIDControllers(
        new PIDController(5.2, 0, 0.4), new PIDController(5, 0, 0.4), new PIDController(1.5, 0, 0));

    VoyagerLib.addEvent(
        "wristdown",
        Commands.runOnce(() -> wrist.requestLowerIntake(false))
            .andThen(Commands.waitUntil(wrist::isLowerIntakeComplete)));

    VoyagerLib.addEvent(
        "wristbang",
        Commands.runOnce(() -> wrist.requestLowerIntake(true))
            .andThen(Commands.waitUntil(wrist::isLowerIntakeComplete)));

    VoyagerLib.addEvent(
        "shoot",
        Commands.startEnd(
                () -> superstructure.setAutoShoot(true), () -> superstructure.setAutoShoot(false))
            .withTimeout(5.5));

    VoyagerLib.addEvent(
        "intake",
        Commands.startEnd(
            () -> superstructure.setAutoIntake(true), () -> superstructure.setAutoIntake(false)));

    VoyagerLib.addEvent("stopintake", Commands.runOnce(() -> superstructure.setAutoIntake(false)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up SysId routines
    autoChooser.addOption("Left Auton", "LeftAuto");
    autoChooser.addOption("Right Auton", "RightAuto");
    autoChooser.addOption("Left Bump Auton", "LeftBumpAuto");
    autoChooser.addOption("Right Bump Auton", "RightBumpAuto");
    autoChooser.addOption("Left Trench Bump Auton", "LeftTrenchBumpAuto");
    autoChooser.addOption("Right Trench Bump Auton", "RightTrenchBumpAuto");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    superstructure.setAutonomous(true);
    return VoyagerLib.runNamedAuto(autoChooser.get());
  }

  public void setAutonomous(boolean enabled) {
    superstructure.setAutonomous(enabled);
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

    updateStateMachineInputs();

    // shooter.setVelocityRPM(shooterRPMset.getAsDouble());

    // Logger.recordOutput("Drive/Angle", drive.getPose().getRotation().getDegrees());
    // Logger.recordOutput(
    //     "Drive/Setpoint", Constants.DriveConstants.ANGLE_PID.getSetpoint().position);
    Logger.recordOutput(
        "Drive/At Goal?",
        (Constants.DriveConstants.ANGLE_PID.atGoal() || drive.getTranslationalSpeed() > 1e-9));

    Logger.recordOutput("Shooter/Regression RPM", superstructure.getRegressionRPM());
    Logger.recordOutput("Shooter/Regression Degrees", superstructure.getRegressionAngleDegrees());
    Logger.recordOutput("Drive/CurrentSpeed", drive.getTranslationalSpeed());

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

  private void updateStateMachineInputs() {
    boolean start = pilot.start().getAsBoolean();
    if (start && !previousStart) {
      drive.setPose(
          new Pose2d(drive.getPose().getTranslation(), AllianceFlip.apply(Rotation2d.kZero)));
    }
    previousStart = start;

    boolean driverEnabled =
        pilot.getCorrectedLeft(Scale.LINEAR).getNorm() != 0.0
            || pilot.getCorrectedRight(Scale.LINEAR).getX() != 0.0
            || pilot.getRightTButton().getAsBoolean()
            || pilot.rightBumper().getAsBoolean()
            || pilot.y().getAsBoolean()
            || pilot.b().getAsBoolean();
    boolean scoring =
        pilot.getRightTButton().getAsBoolean()
            || pilot.y().getAsBoolean()
            || pilot.b().getAsBoolean();
    double distanceFeet = pilot.b().getAsBoolean() ? 3.5 : pilot.y().getAsBoolean() ? 12.0 : 0.0;

    Translation2d target = driverGoal();
    superstructure.updateTeleopInputs(
        driverEnabled,
        -pilot.getCorrectedLeft(Scale.SQUARED).getY(),
        -pilot.getCorrectedLeft(Scale.SQUARED).getX(),
        -pilot.getCorrectedRight(Scale.WILL).getX(),
        target,
        pilot.getRightTButton().getAsBoolean(),
        pilot.getLeftTButton().getAsBoolean(),
        scoring,
        distanceFeet,
        pilot.getLeftTButton().getAsBoolean(),
        copilot.y().getAsBoolean());

    boolean manualWrist = copilot.y().getAsBoolean();
    if (manualWrist) {
      wrist.runWristVolts(4.5 * -copilot.getLeftY(Scale.LINEAR));
    } else if (previousManualWrist) {
      wrist.requestHold();
    }
    previousManualWrist = manualWrist;

    boolean copilotX = copilot.x().getAsBoolean();
    if (copilotX && !previousCopilotX) wrist.stow();
    previousCopilotX = copilotX;

    boolean forceLower = copilot.rightBumper().getAsBoolean();
    if (forceLower && !previousCopilotRightBumper) wrist.requestForceLower();
    previousCopilotRightBumper = forceLower;

    boolean povLeft = copilot.povLeft().getAsBoolean();
    if (povLeft && !previousPovLeft) wrist.requestLowerIntake(false);
    previousPovLeft = povLeft;

    boolean povRight = copilot.povRight().getAsBoolean();
    if (povRight && !previousPovRight) wrist.zero();
    previousPovRight = povRight;

    boolean povUp = copilot.povUp().getAsBoolean();
    if (povUp && !previousPovUp) hood.addOne();
    previousPovUp = povUp;

    boolean povDown = copilot.povDown().getAsBoolean();
    if (povDown && !previousPovDown) hood.subOne();
    previousPovDown = povDown;

    boolean visionToggle = copilot.getRightTButton().getAsBoolean();
    if (visionToggle && !previousVisionToggle) {
      superstructure.setVisionEnabled(!superstructure.isVisionEnabled());
    }
    previousVisionToggle = visionToggle;
  }

  private Translation2d driverGoal() {
    boolean hubSide =
        (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Blue
                && drive.getPose().getX()
                    <= AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX())
            || (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    == DriverStation.Alliance.Red
                && drive.getPose().getX()
                    >= AllianceFlip.apply(FieldConstants.Hub.left_far_corner).getX());

    if (hubSide) return AllianceFlip.apply(FieldConstants.Hub.hub_center_2d);
    return drive
        .getPose()
        .getTranslation()
        .nearest(
            List.of(
                AllianceFlip.apply(FieldConstants.Depot.left_far_corner),
                AllianceFlip.flipY(AllianceFlip.apply(FieldConstants.Depot.left_far_corner))));
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
