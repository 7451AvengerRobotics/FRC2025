package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public Voltage rightVoltage;
        public Voltage leftVoltage;
        public double shooterFeederVoltage = 0.0;
        public boolean intakeBeamBreak = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeIOInputs
     * @see AutoLog
     */
    default void updateInputs(final IntakeIOInputs inputs) {}

    /**
     * Config call, should only be called once
     */
    default void config() {}

    /**
     * Called <b>after</b> {@link IntakeIO#config()}, intended for any initialization that needs to happen post-config
     * and cannot happen pre-config (i.e. in the constructor)
     */
    default void initialize() {}

    default void toVoltage(
            final double rightRollersVolts,
            final double leftRollersVolts
    ) {}

    default void setBeamBreakSensorState(final boolean intakeBeamBroken) {}
}