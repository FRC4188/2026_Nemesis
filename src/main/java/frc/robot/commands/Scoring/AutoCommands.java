package frc.robot.commands.Scoring;

import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.pathbuilder.*;
// import frc.robot.CSPLib.csppathing.PathBuilder;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlip;
import frc.robot.util.FieldConstants;

public class AutoCommands {

  public AutoCommands() {}

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

  public static Swipe curSwipe = Swipe.CENTER;
  public static Start curStart = Start.RIGHT;
  public static Cycle curCycle = Cycle.DOUBLE;

  private static final Drive drive = Drive.getInstance();
  private static final Intake intake = Intake.getInstance();
  private static final Hood hood = Hood.getInstance();
  private static final Shooter shooter = Shooter.getInstance();
  private static final Hopper hopper = Hopper.getInstance();
  private static final Wrist wrist = Wrist.getInstance();

  public static Command constructedAuto = pseudoBoard(curStart, curSwipe, curCycle);

  public static Command autoShoot() {
    return Commands.parallel(
        DriveCommands.joystickCombined(
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () ->
                AllianceFlip.apply(FieldConstants.Hub.hub_center_2d)
                    .minus(drive.getPose().getTranslation())
                    .getAngle(),
            () -> true),
        Commands.runEnd(() -> intake.intakeVolts(1.5), () -> intake.stop()).withTimeout(1),
        ScoringCommands.staticAim(),
        ScoringCommands.staticShoot(),
        ScoringCommands.halfShake());
  }

  public static Command pseudoBoard(Start start, Swipe swipe, Cycle cycle) {
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
              .andThen(AutoCommands.autoShoot())
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
            .andThen(AutoCommands.autoShoot())
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

  public static Command rightDisrupt(
      Drive drive, Intake intake, Hopper hopper, Shooter shooter, Wrist wrist, Hood hood) {
    return Commands.sequence(
            Commands.runOnce(() -> PathBuilder.stopTarget()),
            Commands.runOnce(
                () ->
                    drive.setPose(
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_center, Rotation2d.kZero)))),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                Rotation2d.kZero))
                        .withSpeed(1),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.FuelField.right_midline_corner,
                                Rotation2d.fromDegrees(105)))
                        .withSpeed(0.8)
                        .withRotationSpread(1.5)
                        .withRotationLead(2)
                        .withCommand(
                            () ->
                                Commands.runEnd(() -> intake.intakeVolts(8), intake::stop, intake)),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.FuelField.left_midline_corner,
                                Rotation2d.fromDegrees(105)))
                        .withSpeed(0.8)
                        .withCommand(() -> Commands.runOnce(intake::stop, intake))),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.right_trench_center, 0.4, ScoringCommands.forceDown())),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.left_midline_corner, Rotation2d.kCCW_90deg)),
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
                            Rotation2d.kCCW_90deg),
                        1,
                        0.5),
                    new PathBuilder.Target(new Pose2d(2.225, 2.245, Rotation2d.kCCW_90deg), 0.8)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.right_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d))))),
            Commands.runOnce(() -> PathBuilder.stopTarget()))
        .andThen(AutoCommands.autoShoot().withTimeout(10.0));
  }

  public static Command leftDisrupt(
      Drive drive, Intake intake, Hopper hopper, Shooter shooter, Wrist wrist, Hood hood) {
    return Commands.sequence(
            Commands.runOnce(() -> PathBuilder.stopTarget()),
            Commands.runOnce(
                () ->
                    drive.setPose(
                        AllianceFlip.apply(
                            new Pose2d(
                                FieldConstants.Trench.left_trench_center, Rotation2d.kZero)))),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.left_trench_neutral_preentrance,
                                Rotation2d.kZero))
                        .withSpeed(1),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.FuelField.left_midline_corner,
                                Rotation2d.fromDegrees(-105)))
                        .withSpeed(0.8)
                        .withRotationSpread(1.5)
                        .withRotationLead(2)
                        .withCommand(
                            () ->
                                Commands.runEnd(() -> intake.intakeVolts(8), intake::stop, intake)),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.FuelField.right_midline_corner,
                                Rotation2d.fromDegrees(-105)))
                        .withSpeed(0.8)
                        .withCommand(() -> Commands.runOnce(intake::stop, intake))),
                PathBuilder.triggerWhenFar(
                    FieldConstants.Trench.left_trench_center, 0.4, ScoringCommands.forceDown())),
            Commands.deadline(
                PathBuilder.path(
                    new PathBuilder.Target(
                        new Pose2d(
                            FieldConstants.FuelField.right_midline_corner, Rotation2d.kCW_90deg)),
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
                            Rotation2d.kCW_90deg),
                        1,
                        0.5),
                    new PathBuilder.Target(
                        new Pose2d(2.225, FieldConstants.field_width - 2.245, Rotation2d.kCW_90deg),
                        0.8)),
                PathBuilder.triggerWhenClose(
                    FieldConstants.Trench.left_trench_alliance_preentrance,
                    0.2,
                    Commands.runOnce(
                        () ->
                            PathBuilder.targetTranslation(
                                () -> AllianceFlip.apply(FieldConstants.Hub.hub_center_2d))))),
            Commands.runOnce(() -> PathBuilder.stopTarget()))
        .andThen(AutoCommands.autoShoot().withTimeout(10.0));
  }

  public static Command disruptDoubleSwipe(Start start) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(
                    new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kZero)),
            drive),
        Commands.deadline(
            PathBuilder.path(
                    new PathBuilder.Target(
                            new Pose2d(FieldConstants.Trench.right_trench_center, Rotation2d.kZero))
                        .withCurve(0.4)
                        .withSpeed(0.9),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_neutral_preentrance,
                                Rotation2d.kZero))
                        .withSpeed(0.9),
                    new PathBuilder.Target(
                            new Pose2d(
                                FieldConstants.Trench.right_trench_intermediate.plus(
                                    new Translation2d(0, 0.35)),
                                Rotation2d.fromDegrees(110)))
                        .withRotationLead(2.5)
                        .withRotationSpread(2)
                        .withEndingSpeed(5))
                .andThen(
                    PathBuilder.path(
                        new PathBuilder.Target(
                                new Pose2d(
                                    FieldConstants.Trench.right_trench_intermediate.plus(
                                        new Translation2d(0, 0.35)),
                                    new Rotation2d()))
                            .withStartingSpeed(5)
                            .withStartingRotation(Rotation2d.fromDegrees(120))
                            .withHeading(Rotation2d.fromDegrees(75.270))
                            .withControlDistances(0, 1.024)
                            .withOverrideRotations(
                                new RotationTarget(1, Rotation2d.fromDegrees(121.280)),
                                new RotationTarget(1.97, Rotation2d.fromDegrees(-112.025)),
                                new RotationTarget(2.95, Rotation2d.fromDegrees(-78.298)))
                            .withSpeed(0.8),
                        new PathBuilder.Target(new Pose2d(8.226, 3.520, new Rotation2d()))
                            .withHeading(Rotation2d.fromDegrees(106.045))
                            .withControlDistances(0.719, 0.470)
                            .withSpeed(0.8),
                        new PathBuilder.Target(new Pose2d(6.817, 3.633, new Rotation2d()))
                            .withHeading(Rotation2d.fromDegrees(-119.475))
                            .withControlDistances(0.649, 0.922),
                        new PathBuilder.Target(new Pose2d(6.817, 2.087, new Rotation2d()))
                            .withHeading(Rotation2d.fromDegrees(-102.063))
                            .withControlDistances(0.779, 0.448),
                        new PathBuilder.Target(new Pose2d(5.869, 0.603, new Rotation2d()))
                            .withHeading(Rotation2d.fromDegrees(179.170))
                            .withControlDistances(0.711, 0)
                            .withEndingSpeed(5)
                            .withEndingRotation(Rotation2d.fromDegrees(180)))),
            PathBuilder.triggerWhenFar(
                FieldConstants.Trench.right_trench_center, 0.3, ScoringCommands.forceDown()),
            PathBuilder.triggerWhenClose(
                FieldConstants.FuelField.right_midline_corner,
                0.3,
                Commands.runEnd(() -> intake.intakeVolts(8), intake::stop, intake))),
        Commands.runOnce(intake::stop, intake));
  }

  public static Command fullDepot() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(new Pose2d(3.568, 6.095, Rotation2d.k180deg)), drive),
        PathBuilder.path(
            new PathBuilder.Target(new Pose2d(3.568, 6.095, Rotation2d.k180deg))
                .withHeading(Rotation2d.fromDegrees(-177.030))
                .withOverrideRotations(
                    new RotationTarget(0, Rotation2d.k180deg),
                    new RotationTarget(1, Rotation2d.k180deg))
                .withStartingSpeed(0)
                .withStartingRotation(Rotation2d.k180deg)
                .withControlDistances(0, 1.990)
                .withSpeed(0.5),
            new PathBuilder.Target(new Pose2d(1.115, 5.977, Rotation2d.k180deg))
                .withHeading(Rotation2d.fromDegrees(-174.808))
                .withEndingSpeed(1.4)
                .withEndingRotation(Rotation2d.k180deg)
                .withControlDistances(1.377, 0)),
        new WaitCommand(1.0),
        PathBuilder.path(
            new PathBuilder.Target(new Pose2d(0.728, 5.977, new Rotation2d()))
                .withHeading(Rotation2d.fromDegrees(-0.119))
                .withOverrideRotations(new RotationTarget(0.8, Rotation2d.k180deg))
                .withSpeed(0.5)
                .withStartingSpeed(0)
                .withStartingRotation(Rotation2d.k180deg)
                .withControlDistances(0, 1.020),
            new PathBuilder.Target(new Pose2d(1.675, 5.977, new Rotation2d()))
                .withSpeed(0.5)
                .withHeading(Rotation2d.fromDegrees(-0.044))
                .withControlDistances(0.823, 0.250),
            new PathBuilder.Target(new Pose2d(2.320, 5.041, new Rotation2d()))
                .withHeading(Rotation2d.fromDegrees(115.618))
                .withEndingSpeed(0)
                .withEndingRotation(Rotation2d.fromDegrees(-23.686))
                .withControlDistances(0.383, 0)),
        AutoCommands.autoShoot().withTimeout(10.0));
  }
}
