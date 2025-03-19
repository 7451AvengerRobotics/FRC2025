package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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



    public SuperStructure(
        Intake intake,
        IntakePivot intakePivot,
        Index index,
        Elevator ele,
        ClawPivot clawPivot,
        Claw claw,
        Climber climb
    ) {
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
        return Commands.sequence(
            claw.setClawPower(0),
            index.setIndexPower(0),
            intake.setintakePower(0),
            Commands.parallel(
                climb.setClimberAngle(() -> ClimberPos.STOW),
                ele.toHeightCoral(() -> EleHeight.RESET)
            ),
            Commands.waitUntil(ele::getLimitSwitch),
            clawPivot.pivotClaw(() -> PivotPos.RESET),
            Commands.waitUntil(climb::endClimbCommand),//Change to elevator safe zone
            intakePivot.setIntakePos(() -> IntakePos.STOW)
        );
    }

    public Command intake() {
        return Commands.sequence(
            Commands.parallel(
                intake.setintakePower(1).until(intake::getIntakeBreak),
                intakePivot.setIntakePos(()-> IntakePos.INTAKE)
            ),
            Commands.waitUntil(ele::getLimitSwitch),
            clawPivot.pivotClaw(() -> PivotPos.INTAKE),
            Commands.parallel(
                    intake.setintakePower(0.5),
                    index.setIndexPower(0.7),
                    claw.setClawPower(0.1),
                    intakePivot.setIntakePivotAngle(.2)
            ).until(claw::clawBroke),
            Commands.parallel(
                    intakePivot.setIntakePivotAngle(0),
                    clawPivot.setClawPivotAngle(0.03)
            )
        );
    }

    public Command score() {
        return Commands.sequence(
                claw.setClawPower(-0.1) 
            );
    }

    public Command setL4() {            
        return Commands.sequence(
            ele.toHeightCoral(()-> EleHeight.L4),
            clawPivot.pivotClaw(() -> PivotPos.L4)
        );
    }

    public Command setL3() {            
        return Commands.sequence(
            ele.toHeightCoral(()-> EleHeight.L3),
            clawPivot.pivotClaw(() -> PivotPos.L3)
        );
    }

    public Command setL2() {            
        return Commands.sequence(
            ele.toHeightCoral(()-> EleHeight.L2),
            clawPivot.pivotClaw(() -> PivotPos.L2)
        );
    }

    public Command setL2Algae() {            
        return Commands.sequence(
            ele.toHeightAlgae(()-> AlgaeHeight.L2),
            clawPivot.pivotClaw(() -> PivotPos.L2)
        );
    }

    public Command setL3Algae() {            
        return Commands.sequence(
            ele.toHeightAlgae(()-> AlgaeHeight.L3),
            clawPivot.pivotClaw(() -> PivotPos.L2)
        );
    }

    public Command setBargeAlgae() {            
        return Commands.sequence(
            ele.toHeightAlgae(()-> AlgaeHeight.BARGE),
            clawPivot.pivotClaw(() -> PivotPos.L2)
        );
    }

    public Command setProcessor() {            
        return Commands.sequence(
            ele.toHeightAlgae(()-> AlgaeHeight.PROCESSOR),
            clawPivot.pivotClaw(() -> PivotPos.L2)
        );
    }

    public Command setL1() {            
        return Commands.sequence(
            ele.toHeightCoral(()-> EleHeight.L1)
        );
    }

    public Command setReefLvl(CommandPS5Controller controller) {
        final int povPosition = controller.getHID().getPOV();
        Command reefPosition = setL4();
            if (povPosition == 0) {
                reefPosition = setL4();
            } else if (povPosition == 90) {
                reefPosition = setL3();
            } else if (povPosition == 180) {
                reefPosition = setL2();
            } else if (povPosition == 270) {
                reefPosition = setL1();
            }
            return Commands.sequence(
                Commands.waitUntil(claw::clawBroke),
                Commands.waitUntil(clawPivot::clawClear),
                reefPosition
            );
    }

    public Command setAlgaeLvl(CommandPS5Controller controller) {
        final int povPosition = controller.getHID().getPOV();
        Command reefPosition = setProcessor();
            if (povPosition == 0) {
                reefPosition = setBargeAlgae();
            } else if (povPosition == 90) {
                reefPosition = setL3Algae();
            } else if (povPosition == 180) {
                reefPosition = setL2Algae();
            } else if (povPosition == 270) {
                reefPosition = setProcessor();
            }
            return Commands.sequence(
                Commands.waitUntil(claw::clawBroke),
                Commands.waitUntil(clawPivot::clawClear),
                reefPosition
            );
    }

    public Command readyToClimb() {
        return Commands.parallel(
            intakePivot.setIntakePivotAngle(0.36),
            clawPivot.setClawPivotAngle(0),
            climb.setClimberAngle(-0.22)
        );
    }

    public Command climb() {
        return Commands.parallel(
            clawPivot.setClawPivotAngle(0),
            climb.setClimberPower(0.37)
                .until(climb::endClimbSeq).finallyDo(
                    () -> climb.setAngle(-0.25)
                )
        );
    }
}


