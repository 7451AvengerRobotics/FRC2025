package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {

    private final SparkFlex intake;
    private final RelativeEncoder intakeEncoder;
    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    

    private final TalonFX pivot;
    private TalonFXConfiguration cfg;
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
    private final SysIdRoutine sysId;



    public Intake(){
        super();

        setName("Intake");

        intake = new SparkFlex(IntakeConstants.kIntakeID, MotorType.kBrushless);
        intakeEncoder = intake.getEncoder();

        pivot = new TalonFX(IntakeConstants.kIntakePivotID);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = IntakeConstants.kIntakeGearRatio;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max accel 

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = IntakeConstants.intakePivotkS;
        slot0.kV = IntakeConstants.intakePivotkV;
        slot0.kA = IntakeConstants.intakePivotkA;
        slot0.kP = IntakeConstants.intakePivotkP;
        slot0.kI = IntakeConstants.intakePivotkI;
        slot0.kD = IntakeConstants.intakePivotkD;

        pivot.getConfigurator().apply(cfg);

        

        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,         // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                    null,          // Use default timeout (10 s)
                                           // Log state with Phoenix SignalLogger class
                    state -> SignalLogger.writeString("intakePivotState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    volts -> pivot.setControl(m_sysIdControl.withOutput(volts)),
                    null,
                    this
                )
            );

        SignalLogger.start();
    }

    public void runIntake(double power){
        intake.set(power);
    }

    public void pivotIntake(double angle){
        pivot.setControl(pivotRequest.withPosition(angle).withSlot(0));
    }
    
    public double getEncoderPosition(){
        return intakeEncoder.getPosition(); 
    }

    public Command setintakePower(double power){
        return runEnd(
            () -> {
                this.runIntake(power);
            }, 
            () -> {
                this.runIntake(0);
            });
    }

    public Command setPivotAngle(double angle) {
        return run(() -> {
            this.pivotIntake(angle);
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}