package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.*;

public class Claw extends SubsystemBase {

    private final SparkFlex claw;
    private final RelativeEncoder clawEncoder;
    private final DigitalInput clawBeamBreak = new DigitalInput(4);
    private final TalonFX claw_pivot = new TalonFX(ClawConstants.kClawPivotID);
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

    public Claw(){
        super();

        setName("Claw");

        claw = new SparkFlex(ClawConstants.kClawID, MotorType.kBrushless);
        clawEncoder = claw.getEncoder();

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ClawConstants.kClawGearRatio;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max accel 

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = ClawConstants.clawPivotkS;
        slot0.kV = ClawConstants.clawPivotkV;
        slot0.kA = ClawConstants.clawPivotkA;
        slot0.kP = ClawConstants.clawPivotkP;
        slot0.kI = ClawConstants.clawPivotkI;
        slot0.kD = ClawConstants.clawPivotkD;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = claw_pivot.getConfigurator().apply(cfg);
            if (status.isOK()) 
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        claw_pivot.getConfigurator().setPosition(-0.095);

         BaseStatusSignal.setUpdateFrequencyForAll(250,
            claw_pivot.getPosition(),
            claw_pivot.getVelocity(),
            claw_pivot.getMotorVoltage());


        SignalLogger.start();
    }

    public void run(double power){
        claw.set(power);
    }

    public void pivot(double angle){
        claw_pivot.setControl(pivotRequest.withPosition(angle).withSlot(0));
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

    public Command setClawPivotAngle(double angle) {
        return run(() -> {
            pivot(angle);
        });
    }

    public boolean clawBroke() {
        return !clawBeamBreak.get();
    }

    public BooleanSupplier clawBrokeSupplier() {
        return clawBeamBreak::get;
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Claw Rotations", claw_pivot.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Claw Beam Break", clawBroke());


    }
}