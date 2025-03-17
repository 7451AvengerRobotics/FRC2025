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
import frc.robot.Constants.RobotConstants.ClawConstants;
import frc.robot.subsystems.Elevator.EleHeight;

import static edu.wpi.first.units.Units.*;

public class ClawPivot extends SubsystemBase {

    private final TalonFX claw_pivot = new TalonFX(ClawConstants.kClawPivotID);
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

    public ClawPivot() {
        super();
        setName("clawPivot");
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 40;
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ClawConstants.kClawGearRatio;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.75)) // 5 (mechanism) rotations per second cruise
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

        claw_pivot.getConfigurator().setPosition(-0.061279296875);

         BaseStatusSignal.setUpdateFrequencyForAll(250,
            claw_pivot.getPosition(),
            claw_pivot.getVelocity(),
            claw_pivot.getMotorVoltage());
    }

    public void pivot(double angle){
        claw_pivot.setControl(pivotRequest.withPosition(angle).withSlot(0));
    }

    public Command setClawPivotAngle(double angle, PivotPos pos) {
        return run(() -> {
            pivot(angle);
        }).until(() -> nearSetpoint(pos));
    }

    public Command setClawPivotAngle(double angle) {
        return run(() -> {
            pivot(angle);
        });
    }

    public double getClawPivotPosition() {
        return claw_pivot.getPosition().getValueAsDouble();
    }

    public boolean nearSetpoint(PivotPos pivot) {
        double diff = pivotRequest.Position - pivot.rotations;
        return Math.abs(diff) <= 0.05;
    }

    public boolean clawClear() {
        return getClawPivotPosition() > 0;
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
        L2(6.75097),
        L3(9.529046 - 0.17),
        L4(-0.02),
        INTAKE(2),
        BARGE(5),
        PROCESSOR(0);

        public final double rotations;

        private PivotPos(double rotations){
            this.rotations = rotations;
        }
    }
}
