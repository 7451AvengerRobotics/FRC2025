package frc.robot.subsystems.Claw;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.*;

public class Claw extends SubsystemBase {

    private final SparkFlex claw;
    private final DigitalInput clawBeamBreak = new DigitalInput(4);

    public Claw(){
        super();

        setName("Claw");

        claw = new SparkFlex(ClawConstants.kClawID, MotorType.kBrushless);
    }

    public void run(double power){
        claw.set(power);
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

    public boolean clawBroke() {
        return !clawBeamBreak.get();
    }

    public boolean notClawBroke() {
        return clawBeamBreak.get();
    }

    public boolean motorStall() {
       return (claw.getOutputCurrent() > 30 && claw.getEncoder().getVelocity() == 0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Claw Beam Break", clawBroke());
    }
}