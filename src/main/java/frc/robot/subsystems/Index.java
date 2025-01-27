package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

    private final SparkFlex index;
    private final RelativeEncoder indexEncoder;

    private final TalonFX pivot;

    public Index(){
        super();
        index = new SparkFlex(0, MotorType.kBrushless);
        indexEncoder = index.getEncoder();
        pivot = new TalonFX(24);
    }

    public void run(double power){
        index.set(power);
    }
    
    public double getEncoderPosition(){
        return indexEncoder.getPosition();
    }

    public Command setClawPower(double power){
        return runEnd(
            () -> {
                this.run(power);
            }, 
            () -> {
                this.run(0);
            });
    }
}