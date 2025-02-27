package frc.robot.subsystems.Intake;

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
        SmartDashboard.putBoolean("break", intakebreak.get());
    }
}