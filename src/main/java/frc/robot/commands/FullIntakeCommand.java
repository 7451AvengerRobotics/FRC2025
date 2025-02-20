package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;

public class FullIntakeCommand extends Command {

    private final Intake intake;
    private final Index index;
    private final Claw claw;
    private final double intakePercent;
    private final double indexPercent;
    private final double clawPercent;
    public FullIntakeCommand(Intake intake, Index index, Claw claw, double intakePercent, double indexPercent, double clawPercent){
        this.intake = intake;
        this.index = index;
        this.claw = claw;
        this.intakePercent = intakePercent;
        this.indexPercent = indexPercent;
        this.clawPercent = clawPercent;
  
      
        addRequirements(intake, index, claw);

    }

    @Override
    public void initialize(){
    }
    
    @Override 
    public void execute(){
        if (intake.raiseIntake()) {
            intake.setIntakePivotAngle(30);
            intake.setintakePower(intakePercent)
                .until(intake.coralIntaked())
                .andThen(intake.setIntakePivotAngle(30))
                .andThen(
                    new ParallelCommandGroup(
                        index.setIndexPower(indexPercent),
                        claw.setClawPower(clawPercent),
                        claw.setClawPivotAngle(30)
                    )
                );
        } else {
            intake.setintakePower(intakePercent).until(intake.coralIntaked())
            .andThen(intake.setIntakePivotAngle(clawPercent))
                .andThen(
                    new ParallelCommandGroup(
                        index.setIndexPower(indexPercent),
                        claw.setClawPower(clawPercent),
                        claw.setClawPivotAngle(30)
                    )
                );
        }
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
