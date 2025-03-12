package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Swerve.Drive;

public class AutoRoutines {
    private final Drive drive;
    private final Elevator elevator;
    private final ClawPivot clawPivot;
    private final IntakePivot intakePivot;
    private final Claw claw;
    private final Intake intake;
    private final Index index;

    public AutoRoutines(Drive drive, Elevator elevator, ClawPivot clawPivot, IntakePivot intakePivot, Claw claw,
            Intake intake, Index index) {
        this.drive = drive;
        this.elevator = elevator;
        this.clawPivot = clawPivot;
        this.intakePivot = intakePivot;
        this.claw = claw;
        this.intake = intake;
        this.index = index;
    }

    public Command centerAuto() {
        return Commands.sequence(
                clawPivot.setClawPivotAngle(0.03).until(clawPivot::clawClear),
                Commands.parallel(
                        drive.followPPPathCommand("Center"),
                        (elevator.setElevatorPosition(5.5)
                                .until(
                                        elevator::endCommand)
                                .onlyIf(
                                        clawPivot::clawClear))),
                Commands.parallel(
                        drive.driveToClosestReefScoringFaceWithTranslate(
                                new Transform2d(new Translation2d(0.66, -0.22), new Rotation2d(0))
                        ),
                        clawPivot.setClawPivotAngle(-0.027).until(clawPivot::endCommand)
                ).andThen(claw.setClawPower(0.4).until(claw::notClawBroke)),
                Commands.parallel(
                        drive.followPPPathCommand("AlgaeGrab"),
                        clawPivot.setClawPivotAngle(0.03).withTimeout(0.5)
                ),
                Commands.parallel(
                    elevator.setElevatorPosition(1.4),
                    drive.driveToClosestReefScoringFaceWithTranslate(new Transform2d(new Translation2d(0.48,0), new Rotation2d(0))),
                    claw.setClawPower(0.4).withTimeout(0.75)
                ),
                Commands.parallel(
                    drive.followPPPathCommand("SourceDrive"),
                    claw.setClawPower(0.4)
                )
        );
    }

    public Command processorSide2L4Coral() {
        return Commands.sequence(
                clawPivot.setClawPivotAngle(0.03).until(clawPivot::clawClear),
                Commands.parallel(
                        drive.followPPPathCommand("InitialRightSide"),
                        (elevator.setElevatorPosition(5.5)
                                        .until(
                                                elevator::endCommand)
                                        .onlyIf(
                                                clawPivot::clawClear))),
                Commands.parallel(
                        drive.driveToClosestReefScoringFaceWithTranslate(
                                new Transform2d(new Translation2d(0.66, -0.24), new Rotation2d(0))), clawPivot.setClawPivotAngle(-0.027).until(clawPivot::endCommand))
                        .andThen(claw.setClawPower(0.4).until(claw::notClawBroke)),
                Commands.parallel(drive.followPPPathCommand("ReefToSource1"), Commands.parallel(
                        intakePivot.setIntakePivotAngle(0),
                        elevator.setElevatorPosition(0.0002),
                        clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5).andThen(Commands
                                .parallel(
                                        intake.setintakePower(1),
                                        intakePivot.setIntakePivotAngle(0.36))
                                .until(intake::getIntakeBreak))),
                Commands.parallel(
                        drive.followPPPathCommand("Reef5Path1"),
                        Commands.parallel(
                                intake.setintakePower(0.5),
                                index.setIndexPower(0.7),
                                claw.setClawPower(0.1),
                                clawPivot.setClawPivotAngle(-0.058),
                                intakePivot.setIntakePivotAngle(.2)).until(claw::clawBroke)
                                .andThen(
                                        Commands.parallel(
                                                intakePivot.setIntakePivotAngle(0),
                                                clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5)
                                                .andThen(elevator.setElevatorPosition(5.5)
                                                        .until(
                                                                elevator::endCommand)
                                                        .onlyIf(
                                                                clawPivot::clawClear)
                                                        ))),
                Commands.parallel(drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(new Translation2d(0.72, -0.24), new Rotation2d(0))), clawPivot.setClawPivotAngle(-0.027)
                        .until(clawPivot::endCommand)),
                claw.setClawPower(0.4).until(claw::notClawBroke),
                Commands.parallel(drive.followPPPathCommand("ReefToSource2"), Commands.parallel(
                        intakePivot.setIntakePivotAngle(0),
                        elevator.setElevatorPosition(0.0002),
                        clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5).andThen(Commands
                                .parallel(
                                        intake.setintakePower(1),
                                        intakePivot.setIntakePivotAngle(0.36))
                                .until(intake::getIntakeBreak))),
                Commands.parallel(
                        drive.followPPPathCommand("Reef5Path2"),
                        Commands.parallel(
                                intake.setintakePower(0.5),
                                index.setIndexPower(0.7),
                                claw.setClawPower(0.1),
                                clawPivot.setClawPivotAngle(-0.058),
                                intakePivot.setIntakePivotAngle(.2)).until(claw::clawBroke)
                                .andThen(
                                        Commands.parallel(
                                                intakePivot.setIntakePivotAngle(0),
                                                clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5)
                                                .andThen(elevator.setElevatorPosition(5.5)
                                                        .until(
                                                                elevator::endCommand)
                                                        .onlyIf(
                                                                clawPivot::clawClear)
                                                        .andThen(
                                                                clawPivot.setClawPivotAngle(-0.027)
                                                                        .until(clawPivot::endCommand))))),
                Commands.parallel(drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(new Translation2d(0.72, 0.24), new Rotation2d(0)))),
                claw.setClawPower(0.4).until(claw::notClawBroke));
    }

    public Command processorSide2L2Coral() {
        return Commands.sequence(
                drive.followPPPathCommand("InitialRightSide"),
                clawPivot.setClawPivotAngle(0.03).until(clawPivot::clawClear),
                Commands.parallel(
                        drive.driveToClosestReefScoringFaceWithTranslate(
                                new Transform2d(new Translation2d(0.52, -0.18), new Rotation2d(0))),
                        elevator.setElevatorPosition(2)
                                .until(
                                        elevator::endCommand)
                                .onlyIf(
                                        clawPivot::clawClear))
                        .andThen(claw.setClawPower(0.4).until(claw::notClawBroke)),
                Commands.parallel(drive.followPPPathCommand("ReefToSource1"), Commands.parallel(
                        intakePivot.setIntakePivotAngle(0),
                        elevator.setElevatorPosition(0.0002),
                        clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5).andThen(Commands
                                .parallel(
                                        intake.setintakePower(1),
                                        intakePivot.setIntakePivotAngle(0.36))
                                .until(intake::getIntakeBreak))),
                Commands.parallel(
                        drive.followPPPathCommand("Reef5Path1"),
                        Commands.parallel(
                                intake.setintakePower(0.5),
                                index.setIndexPower(0.7),
                                claw.setClawPower(0.1),
                                clawPivot.setClawPivotAngle(-0.058),
                                intakePivot.setIntakePivotAngle(.2)).until(claw::clawBroke)
                                .andThen(
                                        Commands.parallel(
                                                intakePivot.setIntakePivotAngle(0),
                                                clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5))),
                Commands.parallel(drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(new Translation2d(0.72, -0.24), new Rotation2d(0))),
                elevator.setElevatorPosition(5.5)
                        .until(
                                elevator::endCommand)
                        .onlyIf(
                                clawPivot::clawClear)
                        .andThen(
                                clawPivot.setClawPivotAngle(-0.027).until(clawPivot::endCommand)))
                                        .andThen(claw.setClawPower(0.4).until(claw::notClawBroke)));
    }

    public Command bargeSide2L2Coral() {
        return Commands.sequence(
                drive.followPPPathCommand("BargeScore1"),
                clawPivot.setClawPivotAngle(0.03).until(clawPivot::clawClear),
                Commands.parallel(
                        drive.driveToClosestReefScoringFaceWithTranslate(
                                new Transform2d(new Translation2d(0.52, -0.18), new Rotation2d(0))),
                        elevator.setElevatorPosition(2)
                                .until(
                                        elevator::endCommand)
                                .onlyIf(
                                        clawPivot::clawClear))
                        .andThen(claw.setClawPower(0.4).until(claw::notClawBroke)),
                Commands.parallel(drive.followPPPathCommand("Source1"), Commands.parallel(
                        intakePivot.setIntakePivotAngle(0),
                        elevator.setElevatorPosition(0.0002),
                        clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5).andThen(Commands
                                .parallel(
                                        intake.setintakePower(1),
                                        intakePivot.setIntakePivotAngle(0.36))
                                .until(intake::getIntakeBreak))),
                Commands.parallel(
                        drive.followPPPathCommand("BargeScore2"),
                        Commands.parallel(
                                intake.setintakePower(0.5),
                                index.setIndexPower(0.7),
                                claw.setClawPower(0.1),
                                clawPivot.setClawPivotAngle(-0.058),
                                intakePivot.setIntakePivotAngle(.2)).until(claw::clawBroke)
                                .andThen(
                                        Commands.parallel(
                                                intakePivot.setIntakePivotAngle(0),
                                                clawPivot.setClawPivotAngle(0.03)).withTimeout(0.5))),
                Commands.parallel(drive.driveToClosestReefScoringFaceWithTranslate(
                        new Transform2d(new Translation2d(0.72, -0.24), new Rotation2d(0))),
                elevator.setElevatorPosition(5.5)
                        .until(
                                elevator::endCommand)
                        .onlyIf(
                                clawPivot::clawClear)
                        .andThen(
                                clawPivot.setClawPivotAngle(-0.027).until(clawPivot::endCommand)))
                                        .andThen(claw.setClawPower(0.4).until(claw::notClawBroke)));
    }
}