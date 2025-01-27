package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final SparkFlex intake;
    private final RelativeEncoder intakeEncoder;

    private final TalonFX pivot;

    public Intake(){
        super();
        intake = new SparkFlex(23, MotorType.kBrushless);
        intakeEncoder = intake.getEncoder();
        pivot = new TalonFX(22);
    }

    public void run(double power){
        intake.set(power);
    }

    public void pivot(double angle){
        pivot.setPosition(angle);
    }
    
    public double getEncoderPosition(){
        return intakeEncoder.getPosition(); 
    }

    public Command setintakePower(double power){
        return runEnd(
            () -> {
                this.run(power);
            }, 
            () -> {
                this.run(0);
            });
    }

    public Command setPivotAngle(double angle) {
        return run(() -> {
            this.pivot(angle);
        });
    }
}