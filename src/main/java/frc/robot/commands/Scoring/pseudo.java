package frc.robot.commands.Scoring;

import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pathbuilder.PathBuilder;
import frc.robot.Robot;
import frc.robot.commands.Scoring.AutoCommands.Cycle;
import frc.robot.commands.Scoring.AutoCommands.Start;
import frc.robot.commands.Scoring.AutoCommands.Swipe;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;

public class pseudo {

  public static enum Swipe {
    CENTER,
    CLOSE,
  }

  public static enum Start {
    LEFT,
    RIGHT
  }

  public static enum Cycle {
    NZ,
    NONE,
    DOUBLE
  }

  public static Drive drive = Drive.getInstance();
  public static Intake intake = Intake.getInstance();
  public static Wrist wrist = Wrist.getInstance();

  public static Command pseudoBoard() {
    Start start = Start.LEFT;
    Swipe swipe = Swipe.CLOSE;
    Cycle cycle = Cycle.DOUBLE;

    if (cycle == Cycle.NONE) {
      return Commands.sequence(
          Commands.either(
              Commands.runOnce(
                  () ->
                      drive.setPose(
                          AllianceFlip.apply(
                              switch (start) {
                                case LEFT -> new Pose2d(
                                    FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg);
                                case RIGHT -> new Pose2d(
                                    FieldConstants.Trench.right_trench_center,
                                    Rotation2d.kCCW_90deg);
                              }))),
              Commands.none(),
              () -> true),
          Commands.deadline(
              PathBuilder.path(
                  new PathBuilder.Target(
                          switch (start) {
                            case LEFT -> new Pose2d(
                                FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg);
                            case RIGHT -> new Pose2d(
                                FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg);
                          })
                      .withCurve(0.4),
                  new PathBuilder.Target(
                          switch (start) {
                            case LEFT -> new Pose2d(
                                FieldConstants.Trench.left_trench_neutral_preentrance,
                                Rotation2d.kCW_90deg);
                            case RIGHT -> new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                Rotation2d.kCCW_90deg);
                          })
                      .withCurve(0)
                      .withSpeed(1),
                  new PathBuilder.Target(
                          switch (start) {
                            case LEFT -> new Pose2d(
                                switch (swipe) {
                                  case CENTER -> FieldConstants.Trench.left_trench_intermediate;
                                  case CLOSE -> FieldConstants.Trench
                                      .intake_left_trench_intermediate;
                                },
                                Rotation2d.kCW_90deg);
                            case RIGHT -> new Pose2d(
                                switch (swipe) {
                                  case CENTER -> FieldConstants.Trench.right_trench_intermediate;
                                  case CLOSE -> FieldConstants.Trench
                                      .intake_right_trench_intermediate;
                                },
                                Rotation2d.kCCW_90deg);
                          },
                          0.7)
                      .withCurve(0.4),
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
                            case LEFT -> Rotation2d.kCW_90deg;
                            case RIGHT -> Rotation2d.kCCW_90deg;
                          }),
                      0.25)),
              PathBuilder.triggerWhenFar(
                  switch (start) {
                    case LEFT -> FieldConstants.Trench.left_trench_center;
                    case RIGHT -> FieldConstants.Trench.right_trench_center;
                  },
                  0.25,
                  ScoringCommands.forceDown()),
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
                  Commands.runEnd(() -> intake.intakeVolts(8.5), intake::stop, intake))),
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
                      0.8,
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
                          (start == Start.RIGHT)
                              ? new Pose2d(
                                  FieldConstants.Trench.right_trench_alliance_preentrance,
                                  Rotation2d.kZero)
                              : new Pose2d(
                                  FieldConstants.Trench.left_trench_alliance_preentrance,
                                  Rotation2d.kZero),
                          0.8,
                          0.5)
                      .withRotationMode(PathBuilder.Target.RotationMode.SNAP),
                  new PathBuilder.Target(
                      (start == Start.RIGHT)
                          ? new Pose2d(
                              FieldConstants.Trench.right_trench_alliance_preentrance.plus(
                                  new Translation2d(-0.2, 0.5)),
                              Rotation2d.fromDegrees(50.998))
                          : new Pose2d(
                              FieldConstants.Trench.left_trench_alliance_preentrance.plus(
                                  new Translation2d(-0.2, -0.5)),
                              Rotation2d.fromDegrees(-50.998)),
                      0.5,
                      0.1))),
          Commands.runOnce(() -> PathBuilder.stopTarget())
              .andThen(AutoCommands.autoShoot(AutoCommands.Size.FULL))
              .withTimeout(8),
          // .andThen(ScoringCommands.downNoStall()),
          PathBuilder.path(
              new PathBuilder.Target(
                  (start == Start.RIGHT)
                      ? new Pose2d(
                          FieldConstants.Trench.right_trench_alliance_preentrance.plus(
                              new Translation2d(-0.2, 0.5)),
                          Rotation2d.fromDegrees(50.998))
                      : new Pose2d(
                          FieldConstants.Trench.left_trench_alliance_preentrance.plus(
                              new Translation2d(-0.2, -0.5)),
                          Rotation2d.fromDegrees(-50.998))),
              new PathBuilder.Target(
                      (start == Start.RIGHT)
                          ? new Pose2d(
                              FieldConstants.Trench.right_trench_alliance_preentrance,
                              Rotation2d.kZero)
                          : new Pose2d(
                              FieldConstants.Trench.left_trench_alliance_preentrance,
                              Rotation2d.kZero))
                  .withRotationLead(0.5),
              new PathBuilder.Target(
                      (start == Start.RIGHT)
                          ? new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kZero)
                          : new Pose2d(FieldConstants.Trench.left_trench_center, Rotation2d.kZero))
                  .withRotationLead(0.5)
                  .withCurve(0)));
    }

    // NORMAL CYCLES (NZ OR DOUBLE)

    return Commands.sequence(
        Commands.either(
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
            Commands.none(),
            () -> Robot.isSimulation()),
        Commands.deadline(
            PathBuilder.path(
                new PathBuilder.Target(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_center, Rotation2d.kCW_90deg);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_center, Rotation2d.kCCW_90deg);
                        })
                    .withCurve(0.4)
                    .withSpeed(1),
                new PathBuilder.Target(
                        switch (start) {
                          case LEFT -> new Pose2d(
                              FieldConstants.Trench.left_trench_neutral_preentrance,
                              Rotation2d.kCW_90deg);
                          case RIGHT -> new Pose2d(
                              FieldConstants.Trench.right_trench_neutral_preentrance,
                              Rotation2d.kCCW_90deg);
                        })
                    .withCurve(0)
                    .withSpeed(1),
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
                                case CLOSE -> FieldConstants.Trench
                                    .intake_right_trench_intermediate;
                              },
                              Rotation2d.kCCW_90deg);
                        },
                        1)
                    .withCurve(0.4),
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
                    0.35),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (swipe) {
                          case CENTER -> FieldConstants.field_center.plus(
                              new Translation2d(0, -0.2));
                          case CLOSE -> FieldConstants.FuelField.intake_midline.plus(
                              new Translation2d(0, -0.2));
                        },
                        switch (start) {
                          case LEFT -> Rotation2d.kCW_90deg;
                          case RIGHT -> Rotation2d.kCCW_90deg;
                        }),
                    0.35)),
            PathBuilder.triggerWhenFar(
                switch (start) {
                  case LEFT -> FieldConstants.Trench.left_trench_center;
                  case RIGHT -> FieldConstants.Trench.right_trench_center;
                },
                0.25,
                ScoringCommands.forceDown()),
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
                Commands.runEnd(() -> intake.intakeVolts(7.5), intake::stop, intake))),
        Commands.deadline(
            PathBuilder.path(
                new PathBuilder.Target(
                    new Pose2d(
                        switch (swipe) {
                          case CENTER -> FieldConstants.field_center.plus(
                              new Translation2d(0, -0.2));
                          case CLOSE -> FieldConstants.FuelField.intake_midline.plus(
                              new Translation2d(0, -0.2));
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
                        switch (start) {
                          case LEFT -> Rotation2d.kCW_90deg;
                          case RIGHT -> Rotation2d.kCCW_90deg;
                        }),
                    1,
                    1.5,
                    2),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (start) {
                          case RIGHT -> FieldConstants.Trench.right_trench_neutral_preentrance.plus(
                              new Translation2d(0.5, -0.16));
                          case LEFT -> FieldConstants.Trench.left_trench_neutral_preentrance.plus(
                              new Translation2d(0.5, 0.16));
                        },
                        switch (start) {
                          case LEFT -> Rotation2d.kCW_90deg;
                          case RIGHT -> Rotation2d.kCCW_90deg;
                        })),
                new PathBuilder.Target(
                    new Pose2d(
                        switch (start) {
                          case RIGHT -> FieldConstants.Trench.right_trench_alliance_entrance.plus(
                              new Translation2d(-0.3, -0.18));
                          case LEFT -> FieldConstants.Trench.left_trench_alliance_entrance.plus(
                              new Translation2d(-0.3, 0.18));
                        },
                        switch (start) {
                          case RIGHT -> Rotation2d.fromDegrees(70);
                          case LEFT -> Rotation2d.fromDegrees(-70);
                        }),
                    1,
                    0.5),
                new PathBuilder.Target(
                    new Pose2d(
                        FieldConstants.Trench.right_trench_alliance_entrance.getX() - 1.1,
                        switch (start) {
                          case RIGHT -> FieldConstants.Trench.right_trench_alliance_preentrance
                                  .getY()
                              + 0.15;
                          case LEFT -> FieldConstants.Trench.left_trench_alliance_preentrance.getY()
                              - 0.15;
                        },
                        switch (start) {
                          case RIGHT -> Rotation2d.fromDegrees(70);
                          case LEFT -> Rotation2d.fromRadians(-70);
                        }),
                    1))),
        Commands.runOnce(() -> PathBuilder.stopTarget())
            .andThen(AutoCommands.autoShoot(AutoCommands.Size.FULL))
            .withTimeout(
                switch (cycle) {
                  case NZ -> 8.0;
                  case NONE -> 10.0;
                  case DOUBLE -> 6.0;
                })
            .andThen(Robot.isReal() ? ScoringCommands.downNoStall() : Commands.none()),
        switch (cycle) {
          case NONE -> Commands.none();
          case NZ -> Commands.deadline(
              switch (start) {
                case RIGHT -> PathBuilder.path(
                    new PathBuilder.Target(new Pose2d(1.981, 1.150, Rotation2d.kZero)),
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
                                FieldConstants.FuelField.second_intake_right_close_corner,
                                Rotation2d.kCCW_90deg))
                        .withRotationSpread(2.5)
                        .withRotationLead(1)
                        .withCommand(
                            Commands.runEnd(() -> intake.intakeVolts(8.5), intake::stop, intake)));
                  // .alongWith(ScoringCommands.forceDown());

                case LEFT -> PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(1.981, FieldConstants.field_width - 1.150, Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_alliance_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.Trench.left_trench_neutral_preentrance,
                            Rotation2d.kZero)),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.FuelField.second_intake_left_close_corner,
                                Rotation2d.kCW_90deg))
                        .withRotationSpread(2.5)
                        .withRotationLead(1)
                        .withCommand(
                            Commands.runEnd(() -> intake.intakeVolts(8.5), intake::stop, intake)));
                  // .alongWith(ScoringCommands.downNoStall());
              },
              Commands.none());

          case DOUBLE -> Commands.sequence(
              Commands.deadline(
                  PathBuilder.path(
                          PathBuilder.mirror(
                              () -> (start == Start.LEFT),
                              new PathBuilder.Target(new Pose2d(1.981, 0.5, Rotation2d.kCCW_90deg))
                                  .withCurve(0.6),
                              new PathBuilder.Target(
                                      new Pose2d(
                                          FieldConstants.Trench.right_trench_center.plus(
                                              new Translation2d(0, -0.18)),
                                          Rotation2d.kCCW_90deg))
                                  .withEndingSpeed(5)))
                      // .alongWith(ScoringCommands.downNoStall())
                      .andThen(
                          PathBuilder.path(
                              PathBuilder.mirror(
                                  () -> (start == Start.LEFT),
                                  new PathBuilder.Target(
                                          new Pose2d(
                                              FieldConstants.Trench.right_trench_center.plus(
                                                  new Translation2d(0, -0.18)),
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
                                  new PathBuilder.Target(
                                          new Pose2d(7.355, 1.523 - 0.18, Rotation2d.kZero))
                                      .withHeading(Rotation2d.fromDegrees(66.360))
                                      .withControlDistances(1.517, 0.476),
                                  new PathBuilder.Target(new Pose2d(7.614, 3.051, Rotation2d.kZero))
                                      .withHeading(Rotation2d.fromDegrees(120.689))
                                      .withControlDistances(0.288, 1.250),
                                  new PathBuilder.Target(new Pose2d(5.968, 3.051, Rotation2d.kZero))
                                      .withHeading(Rotation2d.fromDegrees(-104.349))
                                      .withControlDistances(0.955, 0.310),
                                  new PathBuilder.Target(
                                          new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
                                      .withHeading(Rotation2d.fromDegrees(99.792))
                                      .withControlDistances(0.250, 0)
                                      .withEndingRotation(Rotation2d.kZero)
                                      .withEndingSpeed(2)))),
                  Commands.runEnd(() -> intake.intakeVolts(8.5), intake::stop, intake)),
              Commands.runOnce(intake::stop),
              Commands.deadline(
                  PathBuilder.path(
                      PathBuilder.mirror(
                          () -> (start == Start.LEFT),
                          new PathBuilder.Target(new Pose2d(5.968 + 0.2, 0.608, Rotation2d.kZero))
                              .withStartingSpeed(2),
                          new PathBuilder.Target(
                              new Pose2d(
                                  FieldConstants.Trench.right_trench_center.plus(
                                      new Translation2d(0, -0.15)),
                                  Rotation2d.kZero))))));
        });
  }
}
