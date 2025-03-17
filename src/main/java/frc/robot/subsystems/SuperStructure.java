package frc.robot.subsystems;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Claw.ClawPivot.PivotPos;
import frc.robot.subsystems.Elevator.EleHeight;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;

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

    public final Trigger stateTrg_idle = new Trigger(stateEventLoop, () -> m_state == State.IDLE);
    public final Trigger stateTrg_intaking = new Trigger(stateEventLoop, () -> m_state == State.INTAKING);
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



    public SuperStructure(
        Intake intake,
        IntakePivot intakePivot,
        Index index,
        Elevator ele,
        ClawPivot clawPivot,
        Claw claw,
        Climber climb,
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

        trg_teleopL1Req = L1Req;
        trg_teleopL2Req = L2Req;
        trg_teleopL3Req = L3Req;
        trg_teleopL4Req = L4Req;
        trg_teleopScoreReq = scoreReq;
        trg_inIntake = new Trigger(intake::getIntakeBreak);
        trg_inClaw = new Trigger(claw::clawBroke);
        trg_hasCoral = trg_inIntake.or(trg_inClaw);

    }
    public enum State {
        IDLE(0, "idle"),
        INTAKING(1, "intaking"),
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

    /* methods that Actually Do Things */
    public Command resetEverything() {
        return Commands.sequence(
            claw.setClawPower(0),
            index.setIndexPower(0),
            intake.setintakePower(0),
            clawPivot.pivotClaw(() -> PivotPos.RESET),
            Commands.parallel(
                ele.toHeightCoral(() -> EleHeight.RESET),
                climb.setClimberAngle(0)
            ),
            intakePivot.setIntakePivotAngle(0)
        );
    }
    
}
