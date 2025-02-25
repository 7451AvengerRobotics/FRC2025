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

import frc.robot.Constants.RobotConstants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);;
    private final TalonFX intake_pivot = new TalonFX(IntakeConstants.kIntakePivotID);;
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
    private final DigitalInput intakebreak = new DigitalInput(0);

    public Intake(){
        super();

        setName("Intake");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = IntakeConstants.kIntakeGearRatio;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max accel 

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = IntakeConstants.intakePivotkS;
        slot0.kV = IntakeConstants.intakePivotkV;
        slot0.kA = IntakeConstants.intakePivotkA;
        slot0.kP = IntakeConstants.intakePivotkP;
        slot0.kI = IntakeConstants.intakePivotkI;
        slot0.kD = IntakeConstants.intakePivotkD;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intake_pivot.getConfigurator().apply(cfg);
            if (status.isOK()) 
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        intake_pivot.getConfigurator().setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            intake_pivot.getPosition(),
            intake_pivot.getVelocity(),
            intake_pivot.getMotorVoltage());


        SignalLogger.start();
    }

    public void runIntake(double power){
        intake.set(power);
    }

    public boolean getIntakeBreak() {
        return !intakebreak.get();
    }

    public BooleanSupplier getIntakeBreakSupplier() {
        return intakebreak::get;
    }

    public void pivotIntake(double angle){
        intake_pivot.setControl(pivotRequest.withPosition(angle).withSlot(0));
    }

    public Command setintakePower(double power) {
        return runEnd(
            () -> {
                this.runIntake(power);
            }, 
            () -> {
                this.runIntake(0);
            });
    }

    public Command dualIntake(double angle, double power) {
       return run(
        () -> {
            this.runIntake(power);
            this.setIntakePivotAngle(angle);
        }
       );
    }

    public Command setIntakePivotAngle(double angle) {
        return run(() -> {
            this.pivotIntake(angle);
        });
    }

    public boolean raiseIntake() {
        if (intake.getVelocity(true).getValueAsDouble() < 200 && intake.getSupplyCurrent(true).getValueAsDouble() > 100) {
            return true;
        }
        return false;
    }


    @Override
    public void periodic(){

        SmartDashboard.putNumber("Intake Rotations", intake_pivot.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("break", intakebreak.get());

    }
}