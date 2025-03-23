package frc.robot.subsystems.Claw;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.ClawPivotConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

public class ClawPivot extends SubsystemBase {

    private final TalonFX claw_pivot = new TalonFX(ClawPivotConstants.kClawPivotID);
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

    public ClawPivot() {
        super();
        setName("clawPivot");
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 40;
        FeedbackConfigs fdb = cfg.Feedback;

        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.502;
        fdb.SensorToMechanismRatio = ClawPivotConstants.kClawPivotGearRatio;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(2)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max accel 

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = ClawPivotConstants.clawPivotkS;
        slot0.kV = ClawPivotConstants.clawPivotkV;
        slot0.kA = ClawPivotConstants.clawPivotkA;
        slot0.kP = ClawPivotConstants.clawPivotkP;
        slot0.kI = ClawPivotConstants.clawPivotkI;
        slot0.kD = ClawPivotConstants.clawPivotkD;

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
    }

    public void pivot(double angle){
        claw_pivot.setControl(pivotRequest.withPosition(angle).withSlot(0));
    }

    public Command setClawPivotAngle(PivotPos pos) {
        return setClawPivotAngle(pos.clawRotations).until(() -> nearSetpoint(pos));
    }

    public Command setClawPivotAngle(double angle) {
        return run(() -> {
            pivot(angle);
        });
    }

    public Command pivotClaw(Supplier<PivotPos> pos) {
        return setClawPivotAngle(pos.get());

    }

    public double getClawPivotPosition() {
        return claw_pivot.getPosition().getValueAsDouble();
    }

    public boolean atBarge() {
        return nearSetpoint(PivotPos.BARGE);
    }

    public boolean atReset() {
        return nearSetpoint(PivotPos.RESET);
    }

    public boolean nearSetpoint(PivotPos pivot) {
        double diff = claw_pivot.getPosition().getValueAsDouble() - pivot.clawRotations;
        return Math.abs(diff) <= 0.03;
    }

    public boolean clawClear() {
        return getClawPivotPosition() > 0.036 && getClawPivotPosition() < 0.5;
    }

    public boolean getClawVelo() {
        return (getClawPivotPosition() > 0.03 && claw_pivot.getVelocity(true ).getValueAsDouble() == 0);
    }

    public boolean endCommand() {
        if (claw_pivot.getVelocity(true).getValueAsDouble() == 0.0 && (claw_pivot.getPosition().getValueAsDouble() > -0.01 && claw_pivot.getPosition().getValueAsDouble() < -0.005)) {
            return true;
        }
        return false;
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Rotations", claw_pivot.getPosition().getValueAsDouble());
    }
    

    public enum PivotPos {
        L2(0.45),
        L3(0.45),
        L4(0.34),
        INTAKE(-0.003173828125),
        BARGE(0.49),
        RESET(0),
        Algae(0.2),
        INTAKEBALL(0.08),
        PROCESSOR(0.03);

        public final double clawRotations;

        private PivotPos(double clawRotations){
            this.clawRotations = clawRotations;
        }
    }
}
