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
            new ParallelCommandGroup(intake.setintakePower(intakePercent), 
                intake.setIntakePivotAngle(0.38)).until(intake::getIntakeBreak)
            .andThen(intake.setIntakePivotAngle(0.24)).withTimeout(0.5)
                .andThen(
                    new ParallelCommandGroup(
                        index.setIndexPower(indexPercent),
                        claw.setClawPivotAngle(-0.07),
                        claw.setClawPower(clawPercent)
                    ).until(claw::clawBroke)
                );
            }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
