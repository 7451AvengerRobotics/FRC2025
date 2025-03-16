package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);;
    private final DigitalInput intakebreak = new DigitalInput(0);

    public Intake(){

        
        super();

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 40;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = intake.getConfigurator().apply(cfg);
            if (status.isOK()) 
            break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        setName("Intake");
    }

    public void runIntake(double power){
        intake.set(power);
    }

    public boolean getIntakeBreak() {
        return !intakebreak.get();
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

    public boolean raiseIntake() {
        if (intake.getVelocity(true).getValueAsDouble() < 200 && intake.getSupplyCurrent(true).getValueAsDouble() > 100) {
            return true;
        }
        return false;
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("break", getIntakeBreak());
    }
}