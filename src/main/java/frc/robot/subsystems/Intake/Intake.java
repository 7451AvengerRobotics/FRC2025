package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

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
    private final DigitalInput intakebreak = new DigitalInput(6);

    public Intake(){

        
        super();

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 60;

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

    public boolean switchIntake() {
        return this.getIntakeBreak() || this.propIntake();
    }

    public boolean propIntake() {
        return Math.abs(intake.getVelocity().getValueAsDouble()) < 2 && (intake.getStatorCurrent().getValueAsDouble() > 50 && intake.getStatorCurrent().getValueAsDouble() < 60) ;
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


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("break", getIntakeBreak());
        SmartDashboard.putBoolean("stall", propIntake());
        SmartDashboard.putNumber("velo", intake.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("current", intake.getStatorCurrent().getValueAsDouble());
    }
}