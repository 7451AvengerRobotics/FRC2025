package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private final SparkFlex claw;
    private final RelativeEncoder clawEncoder;

    private final TalonFX pivot;

    public Claw(){
        super();
        claw = new SparkFlex(0, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        clawEncoder = claw.getEncoder();
        pivot = new TalonFX(1);
    }

    public void run(double power){
        claw.set(power);
    }

    public void pivot(double angle){
        pivot.setPosition(angle);
    }
    
    public double getEncoderPosition(){
        return clawEncoder.getPosition();
    }

    public Command setClawPower(double power){
        return runEnd(
            () -> {
                run(power);
            }, 
            () -> {
                run(0);
            });
    }

    public Command setPivotAngle(double angle) {
        return run(() -> {
            pivot(angle);
        });
    }
}