// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CSPLib.inputs.CSP_Controller;
import frc.robot.CSPLib.pidtuning.PIDTuning;
import frc.robot.commands.Scoring.LoadingCommands;
import frc.robot.commands.Scoring.ScoringCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.simulation.SimulationVisualizer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Launcher launcher;
  private final Loader loader;
  private final Transfer transfer;
  private final Climber climber;
  private final PIDTuning pidTuner;
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

        launcher = new Launcher(new ShooterIOSim(), new HoodIOSim());
        loader = new Loader(new IntakeIOSim(), new WristIOSim());
        transfer = new Transfer(new HopperIOSim(), new IndexerIOSim());
        climber = new Climber(new ClimberIOSim());

        simvis =
            new SimulationVisualizer(
                "Models", () -> loader.getWristAngle(), () -> launcher.getHoodAngle(), () -> 0);
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
                Constants.Robot::updateAnglePID);
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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
                drive, () -> -pilot.getLeftY(), () -> -pilot.getLeftX(), () -> Rotation2d.kZero));

    // Switch to X swerve alignment when X button is pressed
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

    // pivoting the intake toggle (up/down)
    pilot
        .a()
        .toggleOnTrue(LoadingCommands.pivot(loader, new Rotation2d(Constants.WristConstants.Min_A)))
        .toggleOnFalse(
            LoadingCommands.pivot(loader, new Rotation2d(Constants.WristConstants.Max_A)));

    // intaking button (toggle)
    pilot
        .y()
        .toggleOnTrue(LoadingCommands.load(loader, 12))
        .toggleOnFalse(LoadingCommands.load(loader, 0));

    // aggitating and indexing in one button
    pilot
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                LoadingCommands.aggitate(transfer, 6), LoadingCommands.index(transfer, 6)));

    // sighhhhhhhhhh Will skill issue; Reagan will fix V
    pilot
        .getLeftTButton()
        .onTrue(
            ScoringCommands.WindUp(
                launcher,
                (pilot.getLeftTriggerAxis() > 0.75)
                    ? Constants.ShooterConstants.kHighVel
                    : (pilot.getLeftTriggerAxis() > 0.50)
                        ? Constants.ShooterConstants.kMiddleVel
                        : Constants.ShooterConstants.kLowVel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation() {
    if (Constants.Robot.currentMode != Constants.Mode.SIM) return;

    // SimulatedArena.getInstance().resetFieldForAuto();

   }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.Robot.currentMode != Constants.Mode.SIM) return;

    simvis.update();

    Logger.recordOutput("FieldSimulation/RobotPosition", drive.getPose());
  }
}
