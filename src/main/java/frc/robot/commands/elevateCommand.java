package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawPivot;

public class elevateCommand extends Command {

    private final Elevator elevator;
    private final ClawPivot claw;
    private double position;

    public elevateCommand(Elevator elevator, ClawPivot claw, double position) {
        this.elevator = elevator;
        this.claw = claw;
        this.position = position;

        addRequirements(elevator, claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(claw.getClawPivotPosition() < 0.03) {
            claw.pivot(0.03);
        }

        if (claw.getClawVelo()){
            elevator.elevate(position);

        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    
}

