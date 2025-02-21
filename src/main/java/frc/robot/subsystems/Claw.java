package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants.*;

public class Claw extends SubsystemBase {

    private final SparkFlex claw;
    private final RelativeEncoder clawEncoder;
    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    private final TalonFX claw_pivot = new TalonFX(ClawConstants.kClawPivotID);
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
    private final SysIdRoutine sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second),         // Use default ramp rate (1 V/s)
            Volts.of(1.5), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                   // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("clawPivotState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> claw_pivot.setControl(m_sysIdControl.withOutput(volts)),
            null,
            this
        )
    );

    public Claw(){
        super();

        setName("Claw");

        claw = new SparkFlex(ClawConstants.kClawID, MotorType.kBrushless);
        clawEncoder = claw.getEncoder();

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ClawConstants.kClawGearRatio;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
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

        claw_pivot.getConfigurator().setPosition(0);

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

    public Command clawSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }
    public Command clawSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Claw Rotations", claw_pivot.getPosition().getValueAsDouble());

    }
}