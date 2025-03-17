package frc.robot.subsystems;

import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;

public class SuperStructure {
    private final Elevator ele;
    private final Intake intake;
    private final IntakePivot intakePivot;
    private final ClawPivot clawPivot;
    private final Claw claw;
    private final Climber climb;

    public final EventLoop stateEventLoop = new EventLoop();

    private State m_state = State.IDLE;

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
    
}
