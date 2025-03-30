package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Claw.ClawPivot.PivotPos;
import frc.robot.subsystems.Climber.ClimberPos;
import frc.robot.subsystems.Elevator.AlgaeHeight;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakePivot.IntakePos;

public class SuperStructure {
    private final Elevator ele;
    private final Intake intake;
    private final IntakePivot intakePivot;
    private final ClawPivot clawPivot;
    private final Claw claw;
    private final Climber climb;
    private final Index index;
    private int reefLvl = 1;

    public SuperStructure(
            Intake intake,
            IntakePivot intakePivot,
            Index index,
            Elevator ele,
            ClawPivot clawPivot,
            Claw claw,
            Climber climb) {
        this.intake = intake;
        this.intakePivot = intakePivot;
        this.index = index;
        this.ele = ele;
        this.clawPivot = clawPivot;
        this.claw = claw;
        this.climb = climb;
    }

    /* methods that Actually Do Things */
    public Command resetEverything() {
        return Commands.defer(
            () -> Commands.either(
                stow().alongWith(claw.setClawPower(0).withTimeout(0.1)), 
                reset().alongWith(claw.setClawPower(0).withTimeout(0.1)),
                claw::clawBroke
            ), 
            Set.of(claw, clawPivot, intake, intakePivot, ele, index, climb)
        );
    }

    public Command reset() {
        return Commands.sequence(
                Commands.either(clawPivot.pivotClaw(() -> PivotPos.RESET), clawPivot.pivotClaw(() -> PivotPos.L2),
                        ele::getLimitSwitch),
                Commands.deadline(
                        ele.toHeightCoral(() -> EleHeight.RESET),
                        index.setIndexPower(0),
                        intake.setintakePower(0),
                        climb.setClimberAngle(() -> ClimberPos.STOW)),
                Commands.waitUntil(ele::getLimitSwitch),
                clawPivot.pivotClaw(() -> PivotPos.RESET),
                Commands.waitUntil(climb::endClimbCommand), // Change to elevator safe zone
                intakePivot.setIntakePos(() -> IntakePos.STOW)
        );
    }

    public Command intake(Trigger clawStall) {

        return Commands.defer(
            () -> {
                return Commands.sequence(
                    Commands.either(
                            intakeDown(),
                            Commands.parallel(
                                claw.setClawPower(-1),
                                Commands.parallel(
                                    intake.setintakePower(1).until(intake::getIntakeBreak),
                                    intakePivot.setIntakePos(() -> IntakePos.INTAKE)).until(intake::propIntake)
                                    .andThen(
                                        intakePivot.setIntakePos(() -> IntakePos.INTAKING)
                                        .onlyIf(intake::propIntake)
                                        .alongWith(
                                            intake.setintakePower(1).until(intake::getIntakeBreak)))
                                        .andThen(
                                            intakePivot.setIntakePos(() -> IntakePos.STOW)
                                        )
                            ),
                            clawStall.negate().and(ele::safeToIntake))
                    );
            }, 
        Set.of(claw, clawPivot, intakePivot, intake, index, ele));

    }

    

    public Command intakeDown() {
        return Commands.defer(
            () -> {
                return Commands.sequence(
                    Commands.parallel(
                            intake.setintakePower(1).until(intake::getIntakeBreak),
                            intakePivot.setIntakePos(() -> IntakePos.INTAKE)).until(
                                    intake::propIntake)
                            .andThen(
                                    intakePivot.setIntakePos(() -> IntakePos.INTAKING).onlyIf(intake::propIntake)
                                            .alongWith(
                                                    intake.setintakePower(1).until(intake::getIntakeBreak))),
                    Commands.waitUntil(intake::getIntakeBreak),
                    Commands.parallel(
                        ele.toHeightCoral(() -> EleHeight.INTAKE),
                        clawPivot.pivotClaw(() -> PivotPos.INTAKE)
                    ),
                    Commands.waitUntil(ele::atIntakeSetPoint),
                    Commands.parallel(
                            intake.setintakePower(0.3),
                            index.setIndexPower(0.4),
                            claw.setClawPower(0.85),
                            intakePivot.setIntakePos(() -> IntakePos.INTAKING)).until(claw::clawBroke));
                }, 
                    Set.of(claw, intake, ele, clawPivot, intakePivot, index)
            );
        }

    public Command stow() {
        return Commands.sequence(
                clawPivot.setClawPivotAngle(PivotPos.L2),
                ele.setElevatorPosition(0).until(ele::getLimitSwitch),
                intakePivot.setIntakePos(() -> IntakePos.STOW)
                );
    }

    public Command score() {
        return claw.setClawPower(-0.7);
    }

    public Command scoreL1() {
        return claw.setClawPower(0.6);
    }

    public Command outtakeAlgae() {
        return Commands.sequence(
                claw.setClawPower(0.5));
    }

    public Command setL4() {
        return Commands.sequence(
                ele.toHeightCoral(() -> EleHeight.L4),
                clawPivot.pivotClaw(() -> PivotPos.L4));
    }

    public Command setL3() {
        return Commands.sequence(
                ele.toHeightCoral(() -> EleHeight.L3),
                clawPivot.pivotClaw(() -> PivotPos.L3));
    }

    public Command setL2() {
        return Commands.sequence(
                ele.toHeightCoral(() -> EleHeight.L2),
                clawPivot.pivotClaw(() -> PivotPos.L2));
    }

    public Command setL2Algae() {
        return Commands.sequence(
                clawPivot.pivotClaw(() -> PivotPos.Algae),
                ele.toHeightAlgae(() -> AlgaeHeight.L2),
                claw.setClawPower(-0.5));
    }

    public Command setL3Algae() {
        return Commands.sequence(
                clawPivot.pivotClaw(() -> PivotPos.Algae),
                ele.toHeightAlgae(() -> AlgaeHeight.L3),
                claw.setClawPower(-0.5));
    }

    public Command groundBall() {
        return Commands.sequence(
                ele.toHeightCoral(() -> EleHeight.RESET),
                clawPivot.pivotClaw(() -> PivotPos.INTAKEBALL),
                claw.setClawPower(-0.5));
    }

    public Command processor() {
        return Commands.sequence(
                ele.toHeightCoral(() -> EleHeight.INTAKE),
                clawPivot.pivotClaw(() -> PivotPos.PROCESSOR),
                claw.setClawPower(-0.5));
    }

    public Command setBargeAlgae() {

        return Commands.parallel(
                ele.toHeightAlgae(() -> AlgaeHeight.BARGE),
                clawPivot.pivotClaw(() -> PivotPos.BARGE),
                claw.setClawPower(-0.5)
                );
    }

    public Command setProcessor() {
        return Commands.parallel(
                claw.setClawPower(-0.5),
                Commands.sequence(
                    ele.toHeightAlgae(() -> AlgaeHeight.PROCESSOR),
                    clawPivot.pivotClaw(() -> PivotPos.PROCESSOR)
                )
        );
    }

    public Command setL1() {
        return Commands.parallel(
                ele.toHeightCoral(() -> EleHeight.L1),
                clawPivot.pivotClaw(() -> PivotPos.L1));
    }

    public Command fixIntake() {
        return Commands.parallel(
                intake.setintakePower(-0.5),
                index.setIndexPower(-0.7),
                claw.setClawPower(0.5),
                ele.toHeightCoral(() -> EleHeight.INTAKE),
                clawPivot.pivotClaw(() -> PivotPos.INTAKE),
                intakePivot.setIntakePos(() -> IntakePos.INTAKING)).withTimeout(0.2).andThen(
                        Commands.parallel(
                                intake.setintakePower(0.5),
                                index.setIndexPower(0.5),
                                claw.setClawPower(0.7)).until(claw::clawBroke));
    }

    public Command setReefLvl() {
        return Commands.defer(
                () -> {

                    Command reefPosition = setL2();
                    if (reefLvl == 1) {
                        reefPosition = setL1();
                    } else if (reefLvl == 2) {
                        reefPosition = setL2();
                    } else if (reefLvl == 3) {
                        reefPosition = setL3();
                    } else if (reefLvl == 4) {
                        reefPosition = setL4();
                    }
                    return Commands.sequence(
                            Commands.waitUntil(claw::clawBroke),
                            Commands.waitUntil(clawPivot::clawClear),
                            reefPosition);
                },
                Set.of(ele, clawPivot, claw));
    }

    public void setLvl(CommandPS5Controller controller) {
        final int povPosition = controller.getHID().getPOV();
        if (povPosition > -1) {
            if (povPosition == 90) {
                reefLvl = 1;
            } else if (povPosition == 180) {
                reefLvl = 2;
            } else if (povPosition == 270) {
                reefLvl = 3;
            } else if (povPosition == 0) {
                reefLvl = 4;
            } else {
                reefLvl = 2;
            }
        }

        if (reefLvl < 1 || reefLvl > 4) {
            reefLvl = 4;
        }
    }

    public void printLvl() {
        SmartDashboard.putNumber("Reef Lvl", reefLvl);
    }

    public Command setAlgaeLvl() {
        return Commands.defer(() -> {
            Command reefPosition = setProcessor();
            if (reefLvl == 1) {
                reefPosition = setProcessor();
            } else if (reefLvl == 2) {
                reefPosition = setL2Algae();
            } else if (reefLvl == 3) {
                reefPosition = setL3Algae();
            } else if (reefLvl == 4) {
                reefPosition = setBargeAlgae();
            }
            return Commands.sequence(
                    Commands.waitUntil(claw::notClawBroke),
                    reefPosition                
                );
        },
                Set.of(ele, clawPivot, claw));
    }

    public Command autoSetAlgaeHeight(int reefFace) {
        return Commands.defer(() -> {
            Command reefPosition = setL2Algae();
            if (reefFace == 1 || reefFace == 3 || reefFace == 5) {
                reefPosition = setL2Algae();
            } else if (reefFace == 0 || reefFace == 2 || reefFace == 4) {
                reefPosition = setL3Algae();
            }
            return Commands.sequence(
                    Commands.waitUntil(claw::notClawBroke),
                    reefPosition                
                );
        },
                Set.of(ele, clawPivot, claw));

    }

    public Command readyToClimb() {
        return Commands.parallel(
                intakePivot.setIntakePivotAngle(0.36),
                clawPivot.setClawPivotAngle(0),
                climb.setClimberAngle(-0.22));
    }

    public Command intakeAlgae() {
        return Commands.parallel(
                setL3Algae(),
                claw.setClawPower(-0.85)
        );
    }

    public Command climb() {
        return Commands.parallel(
                clawPivot.setClawPivotAngle(0),
                climb.setClimberPower(0.37)
                        .until(climb::endClimbSeq).finallyDo(
                                () -> climb.setAngle(0.25)));
    }
}
