package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

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

import frc.robot.Constants.RobotConstants.ClimberConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final TalonFX climber = new TalonFX(ClimberConstants.kClimberID);;
    private final MotionMagicVoltage climbRequest = new MotionMagicVoltage(0);

    public Climber(){
        super();

        setName("Climber");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ClimberConstants.kClimberGearRatio;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.32)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0.1)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0.5));
         // Take approximately 0.1 seconds to reach max accel 
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = ClimberConstants.climberkS;
        slot0.kV = ClimberConstants.climberkV;
        slot0.kA = ClimberConstants.climberkA;
        slot0.kP = ClimberConstants.climberkP;
        slot0.kI = ClimberConstants.climberkI;
        slot0.kD = ClimberConstants.climberkD;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = climber.getConfigurator().apply(cfg);
            if (status.isOK()) 
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        climber.getConfigurator().setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            climber.getPosition(),
            climber.getVelocity(),
            climber.getMotorVoltage());

    }

    public void runClimber(double power){
        climber.set(power);
    }

    public void setAngle(double angle){
        climber.setControl(climbRequest.withPosition(angle).withSlot(0));
    }

    public boolean nearSetpoint(ClimberPos pos) {
        double diff = climbRequest.Position - pos.climbRotations;
        return Math.abs(diff) <= 0.05;
    }

    public Command setClimberPower(double power) {
        return runEnd(
            () -> {
                this.runClimber(power);
            }, 
            () -> {
                this.runClimber(0);
            });
    }

    public Command setClimberAngle(double angle) {
        return run(() -> {
            this.setAngle(angle);
        });
    }

    public Command setClimberAngle(ClimberPos pos) {
        return setClimberAngle(pos.climbRotations).until(() -> nearSetpoint(pos));
    }

    public Command setClimberAngle(Supplier<ClimberPos> pos) {
        return setClimberAngle(pos.get());
    }

    public boolean endClimbCommand() {
        if (climber.getVelocity(true).getValueAsDouble() == 0.0 && climber.getPosition().getValueAsDouble() < 0.03) {
            return true;
        }
        return false;
    }

    public boolean endClimbSeq() {
        if ((climber.getPosition().getValueAsDouble() > 0.23 && climber.getPosition().getValueAsDouble() < 0.26)) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Rotations", climber.getPosition().getValueAsDouble());
    }

    public enum ClimberPos {
        STOW(0),
        READY(0.25),
        CLIMBED(-0.25);
        public final double climbRotations;

        private ClimberPos(double climbRotations){
            this.climbRotations = climbRotations;
        }
    }
}