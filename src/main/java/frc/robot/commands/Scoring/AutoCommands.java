package frc.robot.commands.Scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CSPLib.ppp.PathBuilder;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;

public class AutoCommands {

  public static Command autoShoot(
      Drive drive, Intake intake, Hood hood, Shooter shooter, Hopper hopper, Wrist wrist) {
    return Commands.parallel(
        DriveCommands.autonAtAngle(
            drive,
            () ->
                AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                    .minus(drive.getPose().getTranslation())
                    .getAngle()),
        Commands.runEnd(() -> intake.intakeVolts(1.5), () -> intake.stop()).withTimeout(1),
        ScoringCommands.staticAim(drive, hood),
        ScoringCommands.staticShoot(drive, shooter, hopper),
        ScoringCommands.shake(wrist));
  }

  public static enum Swipe {
    CENTER,
    CLOSE,
  }

  public static enum Start {
    LEFT,
    RIGHT
  }

  public static enum Climb {
    CLIMB,
    NONE
  }

  public static Command pseudoBoard(
      Start start,
      Swipe swipe,
      Climb climb,
      Drive drive,
      Shooter shooter,
      Hood hood,
      Hopper hopper,
      Intake intake,
      Wrist wrist,
      Climber climber) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    AllianceFlip.apply(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg);
                        }))),
        Commands.deadline(
            PathBuilder.path(
                new PathBuilder.Target(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg);
                        })
                    .withSpeed(0.8),
                new PathBuilder.Target(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_neutral_preentrance,
                              Rotation2d.kCW_90deg);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_neutral_preentrance,
                              Rotation2d.kCCW_90deg);
                        })
                    .withSpeed(0.8),
                new PathBuilder.Target(
                    switch (start) {
                      case LEFT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.Trench.left_trench_intermediate;
                            case CLOSE -> FieldConstants.Trench.intake_left_trench_intermediate;
                          },
                          Rotation2d.kCW_90deg);
                      case RIGHT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.Trench.right_trench_intermediate;
                            case CLOSE -> FieldConstants.Trench.intake_right_trench_intermediate;
                          },
                          Rotation2d.kCCW_90deg);
                    },
                    0.25),
                new PathBuilder.Target(
                    switch (start) {
                      case LEFT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.FuelField.left_midline_corner;
                            case CLOSE -> FieldConstants.FuelField.intake_left_midline_corner;
                          },
                          Rotation2d.kCW_90deg);
                      case RIGHT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.FuelField.right_midline_corner;
                            case CLOSE -> FieldConstants.FuelField.intake_right_midline_corner;
                          },
                          Rotation2d.kCCW_90deg);
                    },
                    0.25),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (swipe) {
                          case CENTER -> FieldConstants.field_center;
                          case CLOSE -> FieldConstants.FuelField.intake_midline;
                        },
                        switch (start) {
                          case LEFT -> Rotation2d.fromDegrees(-105);
                          case RIGHT -> Rotation2d.fromDegrees(105);
                        }),
                    0.20)),
            PathBuilder.triggerWhenFar(
                switch (start) {
                  case LEFT -> FieldConstants.Trench.left_trench_center;
                  case RIGHT -> FieldConstants.Trench.right_trench_center;
                },
                0.5,
                ScoringCommands.forceDown(wrist)),
            PathBuilder.triggerWhenClose(
                switch (start) {
                  case LEFT -> switch (swipe) {
                    case CENTER -> FieldConstants.FuelField.left_midline_corner;
                    case CLOSE -> FieldConstants.FuelField.intake_left_midline_corner;
                  };
                  case RIGHT -> switch (swipe) {
                    case CENTER -> FieldConstants.FuelField.right_midline_corner;
                    case CLOSE -> FieldConstants.FuelField.intake_right_midline_corner;
                  };
                },
                1,
                Commands.runEnd(() -> intake.intakeVolts(7.0), intake::stop, intake))),
        Commands.deadline(
            PathBuilder.path(
                new PathBuilder.Target(
                    new Pose2d(
                        switch (swipe) {
                          case CENTER -> FieldConstants.field_center;
                          case CLOSE -> FieldConstants.FuelField.intake_midline;
                        },
                        switch (start) {
                          case RIGHT -> Rotation2d.kCCW_90deg;
                          case LEFT -> Rotation2d.kCW_90deg;
                        })),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (start) {
                          case LEFT -> switch (swipe) {
                            case CENTER -> FieldConstants.FuelField.left_midline_corner;
                            case CLOSE -> FieldConstants.FuelField.intake_left_midline_corner;
                          };
                          case RIGHT -> switch (swipe) {
                            case CENTER -> FieldConstants.FuelField.right_midline_corner;
                            case CLOSE -> FieldConstants.FuelField.intake_right_midline_corner;
                          };
                        },
                        Rotation2d.kZero),
                    1,
                    1.5,
                    2),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (start) {
                          case RIGHT -> FieldConstants.Trench.right_trench_neutral_preentrance;
                          case LEFT -> FieldConstants.Trench.left_trench_neutral_preentrance;
                        },
                        Rotation2d.kZero)),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (start) {
                          case RIGHT -> FieldConstants.Trench.right_trench_alliance_preentrance;
                          case LEFT -> FieldConstants.Trench.left_trench_alliance_preentrance;
                        },
                        switch (start) {
                          case RIGHT -> Rotation2d.kCCW_90deg;
                          case LEFT -> Rotation2d.kCW_90deg;
                        }),
                    1,
                    0.5),
                new PathBuilder.Target(
                    new Pose2d(
                        2.225,
                        switch (start) {
                          case RIGHT -> 2.245;
                          case LEFT -> FieldConstants.field_width - 2.245;
                        },
                        switch (start) {
                          case RIGHT -> Rotation2d.kCCW_90deg;
                          case LEFT -> Rotation2d.kCW_90deg;
                        }),
                    0.8)),
            PathBuilder.triggerWhenClose(
                switch (start) {
                  case RIGHT -> FieldConstants.Trench.right_trench_alliance_preentrance;
                  case LEFT -> FieldConstants.Trench.left_trench_alliance_preentrance;
                },
                0.2,
                Commands.runOnce(
                    () ->
                        PathBuilder.targetTranslation(
                            () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)))),
            PathBuilder.triggerWhenClose(
                new Translation2d(
                    2.225,
                    switch (start) {
                      case RIGHT -> 2.245;
                      case LEFT -> FieldConstants.field_width - 2.245;
                    }),
                0.1,
                Commands.runOnce(() -> PathBuilder.stopTarget()))),
        Commands.runOnce(() -> PathBuilder.stopTarget())
            .andThen(AutoCommands.autoShoot(drive, intake, hood, shooter, hopper, wrist))
            .withTimeout(
                switch (climb) {
                  case CLIMB -> 4.0;
                  case NONE -> 6.0;
                }),
        switch (climb) {
          case CLIMB -> Commands.sequence(
              Commands.runOnce(climber::raise, climber),
              switch (start) {
                case RIGHT -> PathBuilder.path(
                    new PathBuilder.Target(new Pose2d(2.225, 2.245, Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Tower.right_approach_pos, Rotation2d.kZero), 0.2),
                    new PathBuilder.Target(
                        new Pose2d(FieldConstants.Tower.right_align_pos, Rotation2d.kZero), 0.1));
                case LEFT -> PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(2.225, FieldConstants.field_width - 2.245, Rotation2d.k180deg),
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
                        new Pose2d(FieldConstants.Tower.left_align_pos, Rotation2d.k180deg), 0.05));
              },
              Commands.runOnce(climber::lower, climber));
          case NONE -> Commands.deadline(
            PathBuilder.path(
                new PathBuilder.Target(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_alliance_preentrance, Rotation2d.kZero);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_alliance_preentrance, Rotation2d.kZero);
                        })
                    .withSpeed(0.8),
                new PathBuilder.Target(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_neutral_preentrance,
                              Rotation2d.kCW_90deg);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_neutral_preentrance,
                              Rotation2d.kCCW_90deg);
                        })
                    .withSpeed(0.8),
                new PathBuilder.Target(
                    switch (start) {
                      case LEFT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.Trench.intake_left_trench_intermediate;
                            case CLOSE -> FieldConstants.Trench.left_trench_intermediate;
                          },
                          Rotation2d.kCW_90deg);
                      case RIGHT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.Trench.intake_right_trench_intermediate;
                            case CLOSE -> FieldConstants.Trench.right_trench_intermediate;
                          },
                          Rotation2d.kCCW_90deg);
                    },
                    0.25),
                new PathBuilder.Target(
                    switch (start) {
                      case LEFT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.FuelField.intake_left_midline_corner;
                            case CLOSE -> FieldConstants.FuelField.left_midline_corner;
                          },
                          Rotation2d.kCW_90deg);
                      case RIGHT -> new Pose2d(
                          switch (swipe) {
                            case CENTER -> FieldConstants.FuelField.intake_right_midline_corner;
                            case CLOSE -> FieldConstants.FuelField.right_midline_corner;
                          },
                          Rotation2d.kCCW_90deg);
                    },
                    0.25),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (swipe) {
                          case CENTER -> FieldConstants.FuelField.intake_midline;
                          case CLOSE -> FieldConstants.field_center;
                        },
                        switch (start) {
                          case LEFT -> Rotation2d.fromDegrees(-105);
                          case RIGHT -> Rotation2d.fromDegrees(105);
                        }),
                    0.20)),
            PathBuilder.triggerWhenClose(
                switch (start) {
                  case LEFT -> switch (swipe) {
                    case CENTER -> FieldConstants.FuelField.intake_left_midline_corner;
                    case CLOSE -> FieldConstants.FuelField.left_midline_corner;
                  };
                  case RIGHT -> switch (swipe) {
                    case CENTER -> FieldConstants.FuelField.intake_right_midline_corner;
                    case CLOSE -> FieldConstants.FuelField.right_midline_corner;
                  };
                },
                1,
                Commands.runEnd(() -> intake.intakeVolts(7.0), intake::stop, intake)));
        });
  }

  public static Command climbRight(Climber climber) {
    return Commands.sequence(
        Commands.runOnce(climber::raise, climber),
        PathBuilder.path(
            new PathBuilder.Target(new Pose2d(2.225, 2.245, Rotation2d.kZero)),
            new PathBuilder.Target(
                new Pose2d(FieldConstants.Tower.right_approach_pos, Rotation2d.kZero), 0.2),
            new PathBuilder.Target(
                new Pose2d(FieldConstants.Tower.right_align_pos, Rotation2d.kZero), 0.1)),
        Commands.runOnce(climber::lower, climber));
  }

  public static Command climbLeft(Climber climber) {
    return Commands.sequence(
        Commands.runOnce(climber::raise, climber),
        PathBuilder.path(
            new PathBuilder.Target(
                new Pose2d(2.225, FieldConstants.field_width - 2.245, Rotation2d.k180deg),
                1,
                0,
                0.5),
            new PathBuilder.Target(
                new Pose2d(FieldConstants.Tower.left_approach_pos, Rotation2d.k180deg), 0.2),
            new PathBuilder.Target(
                new Pose2d(FieldConstants.Tower.left_intermediate_approach_pos, Rotation2d.k180deg),
                0.1),
            new PathBuilder.Target(
                new Pose2d(FieldConstants.Tower.left_inter_inter_approach_pos, Rotation2d.k180deg),
                0.2),
            new PathBuilder.Target(
                new Pose2d(FieldConstants.Tower.left_align_pos, Rotation2d.k180deg), 0.05)),
        Commands.runOnce(climber::lower, climber));
  }
}
