
package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Claw.ClawPivot.PivotPos;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakePivot.IntakePos;
import frc.robot.subsystems.Swerve.Drive;

public class AutoRoutines {
    private final Drive drive;
    private final Elevator elevator;
    private final ClawPivot clawPivot;
    private final IntakePivot intakePivot;
    private final Claw claw;
    private final Intake intake;
    private final Index index;
    private final SuperStructure superStruc;
    

    public AutoRoutines(Drive drive, Elevator elevator, ClawPivot clawPivot, IntakePivot intakePivot, Claw claw,
            Intake intake, Index index, SuperStructure superStruc) {
        this.drive = drive;
        this.elevator = elevator;
        this.clawPivot = clawPivot;
        this.intakePivot = intakePivot;
        this.claw = claw;
        this.intake = intake;
        this.index = index;
        this.superStruc = superStruc;
    }

    

    public Command bargeSideLoli() {
        return Commands.sequence(
                clawPivot.pivotClaw(() -> PivotPos.L2),
                Commands.parallel(superStruc.setL4(), drive.driveToClosestReefScoringFaceWithTranslate(new Transform2d(new Translation2d(0.69, 0.13), 
                    new Rotation2d()))),
                superStruc.score().withTimeout(0.4),
                Commands.parallel(
                        drive.followPPPathCommand("Loli1"),
                        Commands.sequence(
                                superStruc.resetEverything(),
                                intakeMove()
                        )
                ).until(intake::getIntakeBreak),
                Commands.parallel(
                        drive.followPPPathCommand("Score2"),
                        completeIntake().andThen(superStruc.resetEverything())
                ),
                Commands.parallel(superStruc.setL4(), drive.driveToClosestReefScoringFaceWithTranslate(new Transform2d(new Translation2d(0.69, 0.13), 
                    new Rotation2d()))),
                superStruc.score().withTimeout(0.4),
                Commands.parallel(
                        drive.followPPPathCommand("Loli2"),
                        Commands.sequence(
                                superStruc.resetEverything(),
                                intakeMove()
                        )
                ).until(intake::getIntakeBreak),
                Commands.parallel(
                        drive.followPPPathCommand("Score3"),
                        completeIntake().andThen(superStruc.resetEverything())
                ),
                Commands.parallel(superStruc.setL4(), drive.driveToClosestReefScoringFaceWithTranslate(new Transform2d(new Translation2d(0.69, -0.21), 
                    new Rotation2d()))),
                superStruc.score().withTimeout(0.4)
        );
    
    }


    // Helper methods

    private Command intakeMove() {
        return Commands.parallel(
                intake.setintakePower(1).until(intake::getIntakeBreak),
                intakePivot.setIntakePos(() -> IntakePos.INTAKE)).until(intake::propIntake)
                .andThen(
                        intakePivot.setIntakePos(() -> IntakePos.INTAKING)
                        .onlyIf(intake::propIntake)
                        .alongWith(
                                intake.setintakePower(1).until(intake::getIntakeBreak)));
    }

    private Command completeIntake() {
        return Commands.sequence(
                Commands.parallel(
                        elevator.toHeightCoral(() -> EleHeight.INTAKE),
                        clawPivot.pivotClaw(() -> PivotPos.INTAKE)
                ),
                Commands.waitUntil(elevator::atIntakeSetPoint),
                Commands.parallel(
                        intake.setintakePower(0.3),
                        index.setIndexPower(0.4),
                        claw.setClawPower(0.85),
                        intakePivot.setIntakePos(() -> IntakePos.INTAKING)).until(claw::clawBroke)
        );
    }


}

