package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.IndexConstants;

public class Index extends SubsystemBase {

    private final TalonFX index;
    
    public Index(){
        super();
        index = new TalonFX(IndexConstants.kIndexID);
    }

    public void runIndex(double power){
        index.set(power);
    }

    public Command setIndexPower(double power){
        return runEnd(
            () -> {
                this.runIndex(power);
            }, 
            () -> {
                this.runIndex(0);
            });
    }
}