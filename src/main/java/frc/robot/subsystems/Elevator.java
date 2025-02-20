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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants.*;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator = new TalonFX(ElevatorConstants.kElevatorID);;
    private final MotionMagicVoltage elevatorRequest = new MotionMagicVoltage(0);
    private final VoltageOut m_sysIdControl = new VoltageOut(0);
    private final SysIdRoutine sysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                   // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("elevatorState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> elevator.setControl(m_sysIdControl.withOutput(volts)),
            null,
            this
        )
    );
    
    public Elevator(){
        super();

        setName("Elevator");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = ElevatorConstants.kElevatorGearRatio;
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // 5 (mechanism) rotations per second cruise
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
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

        BaseStatusSignal.setUpdateFrequencyForAll(250,
            elevator.getPosition(),
            elevator.getVelocity(),
            elevator.getMotorVoltage());

        elevator.optimizeBusUtilization(); //make sure to remove after sysid

        SignalLogger.start();
    }

    public void elevate(double rotations){
        elevator.setControl(elevatorRequest.withPosition(rotations).withSlot(0));
    }
    

    public Command setElevatorPosition(double rotations) {
        return run(() -> {
            elevate(rotations);
        });
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }
    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("Elevator Rotations", elevator.getPosition().getValueAsDouble());

    }
}