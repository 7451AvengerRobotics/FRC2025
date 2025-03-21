package frc.robot.subsystems.Claw;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.*;

public class Claw extends SubsystemBase {

    private final SparkFlex claw;
    private final DigitalInput clawBeamBreak = new DigitalInput(9);
    private SparkFlexConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public Claw(){
        super();

        setName("Claw");

        claw = new SparkFlex(ClawConstants.kClawID, MotorType.kBrushless);

        closedLoopController = claw.getClosedLoopController();
        encoder = claw.getEncoder();

        motorConfig = new SparkFlexConfig();

        motorConfig.inverted(true);

        motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(1000)
        .maxAcceleration(1000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot0)
        .maxVelocity(6000, ClosedLoopSlot.kSlot0)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0);

        claw.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
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
       return (claw.getOutputCurrent() > 50);
    }

    public boolean notStalled() {
        return !motorStall();
    }

    public void holdWhenStall() {

        if (motorStall()) {
            double targetPosition = encoder.getPosition();
            closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0);
        }

    }

    public Command setClawHold() {
        return run(() -> {
            holdWhenStall();
        });
    }

    public Command intakeAlgae(double power) {
        return
            setClawPower(power).until(this::motorStall).andThen(
                Commands.defer(
                    () -> setClawHold(), 
                    Set.of(this)
                )
            );
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Claw Beam Break", clawBroke());
        SmartDashboard.putNumber("Claw Current", claw.getOutputCurrent());
    }
}