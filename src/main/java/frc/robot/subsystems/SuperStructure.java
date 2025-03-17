package frc.robot.subsystems;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Claw.ClawPivot.PivotPos;
import frc.robot.subsystems.Climber.ClimberPos;
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

    public final EventLoop stateEventLoop = new EventLoop();

    private State m_state = State.IDLE;

    private final Trigger trg_teleopL1Req; 
    private final Trigger trg_teleopL2Req; 
    private final Trigger trg_teleopL3Req; 
    private final Trigger trg_teleopL4Req; 
    private final Trigger trg_teleopScoreReq;
    private final Trigger trg_intakeTrigger;

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
    public final Trigger stateTrg_heldInIntake = new Trigger(stateEventLoop, () -> m_state == State.ININTAKE);
    public final Trigger stateTrg_intook = new Trigger(stateEventLoop, () -> m_state == State.INTOOK);
    public final Trigger stateTrg_eleToL1 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L1);
    public final Trigger stateTrg_eleToL2 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L2);
    public final Trigger stateTrg_eleToL3 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L3);
    public final Trigger stateTrg_eleToL4 = new Trigger(stateEventLoop, () -> m_state == State.ELE_TO_L4);
    public final Trigger stateTrg_scoreReady = new Trigger(stateEventLoop, () -> m_state == State.SCORE_READY);
    public final Trigger stateTrg_scoring = new Trigger(stateEventLoop, () -> m_state == State.SCORING);
    public final Trigger stateTrg_scored = new Trigger(stateEventLoop, () -> m_state == State.SCORED);

    public final Trigger stateTrg_climbReady = new Trigger(stateEventLoop, () -> m_state == State.CLIMB_READY);
    public final Trigger stateTrg_climbing = new Trigger(stateEventLoop, () -> m_state == State.CLIMBING);
    public final Trigger stateTrg_climbed = new Trigger(stateEventLoop, () -> m_state == State.CLIMBED);
    private final Trigger trg_hasCoral;
    private final Trigger trg_inIntake;
    private final Trigger trg_inClaw;
    private final Trigger trg_readyToScore;
    private final Trigger trg_holdInIntake;



    public SuperStructure(
        Intake intake,
        IntakePivot intakePivot,
        Index index,
        Elevator ele,
        ClawPivot clawPivot,
        Claw claw,
        Climber climb,
        Trigger IntakeReq,
        Trigger L1Req,
        Trigger L2Req,
        Trigger L3Req,
        Trigger L4Req,
        Trigger scoreReq
    ) {
        this.intake = intake;
        this.intakePivot = intakePivot;
        this.index = index;
        this.ele = ele;
        this.clawPivot = clawPivot;
        this.claw = claw;
        this.climb = climb;

        trg_intakeTrigger = IntakeReq;
        trg_teleopL1Req = L1Req;
        trg_teleopL2Req = L2Req;
        trg_teleopL3Req = L3Req;
        trg_teleopL4Req = L4Req;
        trg_teleopScoreReq = scoreReq;
        trg_inIntake = new Trigger(intake::getIntakeBreak);
        trg_inClaw = new Trigger(claw::clawBroke);
        trg_hasCoral = trg_inIntake.or(trg_inClaw);
        trg_readyToScore = new Trigger(ele::endCommand).and(clawPivot::endCommand);
        trg_holdInIntake = new Trigger(ele::getLimitSwitch);

        configureStateTransitions();
        configureStateActions();

    }

    /* methods that Actually Do Things */
    public Command resetEverything() {
        return Commands.sequence(
            claw.setClawPower(0),
            index.setIndexPower(0),
            intake.setintakePower(0),
            clawPivot.pivotClaw(() -> PivotPos.RESET),
            Commands.parallel(
                climb.setClimberAngle(() -> ClimberPos.STOW),
                ele.toHeightCoral(() -> EleHeight.RESET)
            ),
            Commands.waitUntil(climb::endClimbCommand),
            intakePivot.setIntakePos(() -> IntakePos.STOW)
        );
    }

    public Command intake() {
        return Commands.sequence(
            Commands.parallel(
                intake.setintakePower(1).until(intake::getIntakeBreak),
                intakePivot.setIntakePos(()-> IntakePos.INTAKE)),
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

    private Command changeStateCmd(State newState) {
        return Commands.runOnce(() -> {
            if (newState == m_state) {
                return;
            }
            m_state = newState;
        });
    }

    private void configureStateTransitions() {
        (stateTrg_idle.and(trg_intakeTrigger).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.INTAKING));
        (stateTrg_intaking.and(trg_intakeTrigger).and(trg_holdInIntake.negate()).and(trg_inIntake).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ININTAKE));
        (stateTrg_heldInIntake.and(trg_inIntake).and(trg_holdInIntake))
            .onTrue(changeStateCmd(State.INTOOK));
        (trg_inClaw.and(trg_teleopL1Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L1));
        (trg_inClaw.and(trg_teleopL2Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L2));
        ((trg_inClaw).and(trg_teleopL3Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L3));
        (trg_hasCoral.and(trg_teleopL4Req).and(RobotModeTriggers.teleop()))
            .onTrue(changeStateCmd(State.ELE_TO_L4));
        (stateTrg_eleToL1.and(trg_readyToScore))
            .onTrue(changeStateCmd(State.SCORE_READY)); 
        (stateTrg_eleToL2.and(trg_readyToScore))
            .onTrue(changeStateCmd(State.SCORE_READY)); 
        (stateTrg_eleToL3.and(trg_readyToScore))
            .onTrue(changeStateCmd(State.SCORE_READY)); 
        (stateTrg_eleToL4.and(trg_readyToScore))
            .onTrue(changeStateCmd(State.SCORE_READY)); 
        (stateTrg_scoreReady.and(trg_teleopScoreReq).and(RobotModeTriggers.teleop())) 
            .onTrue(changeStateCmd(State.SCORING));
        (stateTrg_scoring.and(trg_inClaw.negate())) 
            .onTrue(changeStateCmd(State.SCORED));
        (stateTrg_scored.debounce(0.05))
            .onTrue(changeStateCmd(State.IDLE));
    }

    private void configureStateActions() {
        stateTrg_idle.onTrue(resetEverything());

        stateTrg_intaking.onTrue(
                Commands.sequence(
                    intakePivot.setIntakePos(() -> IntakePos.INTAKE),
                    intake.setintakePower(1)
                )
            );
        
        stateTrg_heldInIntake.onTrue(
                Commands.sequence(
                    intakePivot.setIntakePos(() -> IntakePos.INTAKE),
                    intake.setintakePower(0)
                ).until(trg_inIntake)
            );
        
        stateTrg_intook.onTrue(
            Commands.sequence(
                intake.setintakePower(0.5),
                index.setIndexPower(0.7),
                claw.setClawPower(0.1),
                clawPivot.pivotClaw(()-> PivotPos.INTAKE),
                intakePivot.setIntakePos(()-> IntakePos.INTAKING)
            ).until(trg_inClaw)
        );

        stateTrg_eleToL1.onTrue(
            Commands.sequence(
                ele.toHeightCoral(()-> EleHeight.L1)
            )
        );

        stateTrg_eleToL2.onTrue(
            Commands.sequence(
                ele.toHeightCoral(()-> EleHeight.L2)
            )
        );

        stateTrg_eleToL3.onTrue(
            Commands.sequence(
                ele.toHeightCoral(()-> EleHeight.L3)
            )
        );

        stateTrg_eleToL4.onTrue(
            Commands.sequence(
                ele.toHeightCoral(()-> EleHeight.L4),
                clawPivot.pivotClaw(() -> PivotPos.L4)
            )
        );

        stateTrg_scoreReady.onTrue(
            Commands.sequence(
                clawPivot.setClawPivotAngle(PivotPos.L4.clawRotations - 0.2)
            )
        );
        stateTrg_scoring.onTrue(
            Commands.sequence(
                claw.setClawPower(-0.1) 
            )
        );
    }

    public Command forceIdle() {
        return (changeStateCmd(State.IDLE));
    }

    public Command forcetoHP() {
        return (changeStateCmd(State.ELE_TO_HP));
    }
    public Command forceStateToIntake() {
        return (changeStateCmd(State.INTAKING));
    }
   public Command forceShoot() {
        return m_coral.score();
    }
    public Command changeStateToScored() {
        return (changeStateCmd(State.SCORED));
    }
    public Command forceL1() {
        return (changeStateCmd(State.ELE_TO_L1));
    }
    public Command forceL2() {
        return (changeStateCmd(State.ELE_TO_L2));
    }
    public Command forceL3() {
        return (changeStateCmd(State.ELE_TO_L3));
    }
    public Command forceL4() {
        return (changeStateCmd(State.ELE_TO_L4));
    }

    public enum State {
        IDLE(0, "idle"),
        INTAKING(1, "intaking"),
        ININTAKE(1.1, "In the Intake"),
        INTOOK(2, "intook"),
        ELE_TO_L1(3.1, "ele to L1"),
        ELE_TO_L2(3.2, "ele to L2"),
        ELE_TO_L3(3.3, "ele to L3"),
        ELE_TO_L4(3.4, "ele to L4"),
        SCORE_READY(4, "score ready"),
        SCORING(5, "scoring"),
        SCORED(6, "scored"),
        CLIMB_READY(7, "climb ready"),
        CLIMBING(8, "climbing"),
        CLIMBED(9, "climbed");

        public final double idx;
        public final String name;
  
        private State(double index, String _name) {
            idx = index;
            name = _name;
        }
    }
}


