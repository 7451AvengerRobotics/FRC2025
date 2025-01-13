package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
    @SuppressWarnings("unused")
    private final IntakeConstants intakeConstants;

    private final TalonFX rightRoller;
    private final TalonFX leftRoller;

    private final DigitalInput intakeBeamBreak;
    private final VoltageOut voltageOut;

    private final StatusSignal<Voltage> _rightRollerVoltage;
    private final StatusSignal<Voltage> _leftRollerVoltage;
    
    
        public IntakeIOReal(final Constants.IntakeConstants intakeConstants) {
            this.intakeConstants = intakeConstants;

            this.rightRoller = new TalonFX(intakeConstants.rightRollerMotor());
        this.leftRoller = new TalonFX(intakeConstants.leftRollerMotor());

        this.intakeBeamBreak = new DigitalInput(1);

        this.voltageOut = new VoltageOut(0);

        this._rightRollerVoltage = rightRoller.getMotorVoltage();

        this._leftRollerVoltage = leftRoller.getMotorVoltage();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                _rightRollerVoltage,
                _leftRollerVoltage
        );


        inputs.rightVoltage = _rightRollerVoltage.getValue();

        inputs.leftVoltage = _leftRollerVoltage.getValue();

        inputs.intakeBeamBreak = !intakeBeamBreak.get();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration();
        rightRollerConfig.Slot0 = new Slot0Configs()
                .withKS(3.3326)
                .withKV(0.15104)
                .withKA(0.2004)
                .withKP(10.746);
        rightRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        rightRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        rightRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        rightRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightRoller.getConfigurator().apply(rightRollerConfig);

        final TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration();
        leftRollerConfig.Slot0 = new Slot0Configs()
                .withKS(8.747)
                .withKV(0.12886)
                .withKA(0.27559)
                .withKP(10.901);
        leftRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        leftRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        leftRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftRoller.getConfigurator().apply(leftRollerConfig);


        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                _rightRollerVoltage,
                _leftRollerVoltage
        );

        ParentDevice.optimizeBusUtilizationForAll(
                rightRoller,
                leftRoller
        );
    }

    @Override
    public void toVoltage(double rightRollerVolts, double leftRollerVolts) {
        rightRoller.setControl(voltageOut.withOutput(rightRollerVolts));
        leftRoller.setControl(voltageOut.withOutput(leftRollerVolts));
    }
}