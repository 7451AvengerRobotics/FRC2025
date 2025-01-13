package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;


    private final EventLoop eventLoop;
    public final Trigger intakeBeamBroken;

    private boolean intakingActive = false;
    public final Trigger intaking;

    private static class VelocitySetpoint {
        public double rightRollerVelocityRotsPerSec;
        public double leftRollerVelocityRotsPerSec;
        public double shooterFeederRotsPerSec;
    }

    private final SysIdRoutine torqueCurrentSysIdRoutine;

    public Intake(  
            final Constants.RobotMode robotMode,
            final Constants.IntakeConstants intakeConstants
    ) { 
        this.intakeIO = switch (robotMode) {
            case REAL -> new IntakeIOReal(intakeConstants);
            case SIM -> new IntakeIOSim(intakeConstants);
            case REPLAY, DISABLED -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();

        this.eventLoop = new EventLoop();
        this.intakeBeamBroken = new Trigger(eventLoop, () -> inputs.intakeBeamBroken);

        this.intaking = new Trigger(eventLoop, () -> this.intakingActive);

        this.intakeIO.config();
        this.intakeIO.initialize();
    }

    @SuppressWarnings("deprecation")
    @Override
    public void periodic() {
        @SuppressWarnings("deprecation")
        final double intakeIOPeriodicStart = Logger.getRealTimestamp();
        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        eventLoop.poll();

        Logger.recordOutput(LogKey + "/Intaking", intaking.getAsBoolean());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(Logger.getRealTimestamp() - intakeIOPeriodicStart)
        );
    }

    public Command intakeCommand() {
        return Commands.sequence(
                runOnce(() -> this.intakingActive = true),
                runVoltageCommand(12, 12)
                        .until(intakeBeamBroken),
                instantStopCommand()
//                storeCommand()
        ).finallyDo(() -> this.intakingActive = false);
    }

    public Command runEjectOutCommand() {
        return runVoltageCommand(-12, -12);
    }

    public Command runEjectInCommand() {
        return runVoltageCommand(12, 12);
    }

    public Command instantStopCommand() {
        return runOnce(() -> {
            intakeIO.toVoltage(
                    0,
                    0
            );
        });
    }

    public Command runStopCommand() {
        return runVoltageCommand(0, 0);
    }

    public Command runVoltageCommand(
            final double rightRollerVoltage,
            final double leftRollerVoltage
    ) {
        return startEnd(
                () -> intakeIO.toVoltage(
                        rightRollerVoltage,
                        leftRollerVoltage
                ),
                () -> intakeIO.toVoltage(0, 0)
        );
    }

    /**
     * Set sensor state. No-op if not in simulation.
     * @param shooterBeamBroken whether the shooter beam break is broken.
     */
    public void setBeamBreakSensorState(final boolean shooterBeamBroken) {
        intakeIO.setBeamBreakSensorState(shooterBeamBroken);
    }

}