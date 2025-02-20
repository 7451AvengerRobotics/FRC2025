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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase {

    private final TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);;
    private final DigitalInput beamBreak = new DigitalInput(0);
    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    private final TalonFX intake_pivot = new TalonFX(IntakeConstants.kIntakePivotID);;
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
    private final SysIdRoutine sysId  =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(1.5), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                   // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("intakePivotState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> intake_pivot.setControl(m_sysIdControl.withOutput(volts)),
            null,
            this
        )
    );

    public Intake(){
        super();

        setName("Intake_Pivot");

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

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intake_pivot.getConfigurator().apply(cfg);
            if (status.isOK()) 
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            intake_pivot.getPosition(),
            intake_pivot.getVelocity(),
            intake_pivot.getMotorVoltage());


        SignalLogger.start();
    }

    public void runIntake(double power){
        intake.set(power);
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

    public Command intakeSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }
    public Command intakeSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    public BooleanSupplier coralIntaked() {
        return beamBreak::get;
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Intake Rotations", intake_pivot.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Intake Beam Break", coralIntaked().getAsBoolean());

    }
}