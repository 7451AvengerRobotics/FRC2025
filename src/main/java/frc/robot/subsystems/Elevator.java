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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.*;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator = new TalonFX(ElevatorConstants.kElevatorID);
    private final DigitalInput limitSwitch = new DigitalInput(2);
    private final MotionMagicVoltage elevatorRequest = new MotionMagicVoltage(0);

    public Elevator(){
        super();

        setName("Elevator");


        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 40;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.51;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ElevatorConstants.kElevatorGearRatio;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(20)) // Take approximately 0.5 seconds to reach max vel
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max accel 

        Slot0Configs slot0 = cfg.Slot0;

        

        slot0.kG = ElevatorConstants.elevatorkG;
        slot0.kS = ElevatorConstants.elevatorkS;
        slot0.kV = ElevatorConstants.elevatorkV;
        slot0.kA = ElevatorConstants.elevatorkA;
        slot0.kP = ElevatorConstants.elevatorkP;
        slot0.kI = ElevatorConstants.elevatorkI;
        slot0.kD = ElevatorConstants.elevatorkD;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevator.getConfigurator().apply(cfg);
            if (status.isOK()) 
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        elevator.getConfigurator().setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            elevator.getPosition(),
            elevator.getVelocity(),
            elevator.getMotorVoltage());
    }

    public void elevate(double rotations){
        elevator.setControl(elevatorRequest.withPosition(rotations).withSlot(0));
    }

    public boolean endCommand() {
        if (elevator.getVelocity(true).getValueAsDouble() == 0.0 && elevator.getPosition().getValueAsDouble() > 0.1) {
            return true;
        }
        return false;
    }  

    public Command setElevatorPosition(EleHeight height) {
        return setElevatorPosition(height.rotations).until(() -> nearSetpoint(height));
    }

    public Command setElevatorPosition(AlgaeHeight height) {
        return setElevatorPosition(height.rotations).until(() -> nearSetpoint(height));
    }

    public Command setElevatorPosition(double rotations) {
        return run(() -> {
            elevate(rotations);
        });
    }

    public Command toHeightCoral(Supplier<EleHeight> height) {
        return setElevatorPosition(height.get());
    }

    public Command toHeightAlgae(Supplier<AlgaeHeight> height) {
        return setElevatorPosition(height.get());
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    public boolean elevatorReset() {
        return (elevator.getPosition().getValueAsDouble() < 0.1) && elevator.getVelocity(true).getValueAsDouble() == 0.0;
    }

    public boolean nearSetpoint(EleHeight height) {
        double diff = elevatorRequest.Position - height.rotations;
        return Math.abs(diff) <= 0.05;
    }

    public boolean nearSetpoint(AlgaeHeight height) {
        double diff = elevatorRequest.Position - height.rotations;
        return Math.abs(diff) <= 0.05;
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Elevator Rotations", elevator.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("limit Switch", getLimitSwitch());
    }

    public enum EleHeight {
        RESET(0),
        L1(5.590325),
        L2(5.653564 + 0.169),
        L3(8.451660 + (0.169 / 2)),
        L4(12.89);

        public final double rotations;

        private EleHeight(double rotations){
            this.rotations = rotations;
        }
    }

    public enum AlgaeHeight {
        L2(6.75097),
        L3(9.529046 - 0.17),
        BARGE(5),
        PROCESSOR(0);

        public final double rotations;

        private AlgaeHeight(double rotations){
            this.rotations = rotations;
        }
    }
}