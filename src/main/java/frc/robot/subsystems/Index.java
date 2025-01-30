package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.IndexConstants;

public class Index extends SubsystemBase {

    private final SparkFlex index;
    private final RelativeEncoder indexEncoder;
    
    public Index(){
        super();
        index = new SparkFlex(IndexConstants.kIndexID, MotorType.kBrushless);
        indexEncoder = index.getEncoder();
    }

    public void runIndex(double power){
        index.set(power);
    }
    
    public double getEncoderPosition(){
        return indexEncoder.getPosition();
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