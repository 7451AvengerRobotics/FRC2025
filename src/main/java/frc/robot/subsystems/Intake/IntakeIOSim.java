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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.closeables.ToClose;
import frc.robot.util.control.DeltaTime;
import frc.robot.util.sim.SimUtils;
import frc.robot.util.sim.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    @SuppressWarnings("unused")
private final IntakeConstants intakeConstants;
    private final DeltaTime deltaTime;

    private final TalonFX rightRoller;
    private final TalonFX leftRoller;

    private final DigitalInput intakeBeamBreak;
    private final DIOSim intakeBeamBrakeSim;

    private final TalonFXSim rightRollerSim;
    private final TalonFXSim leftRollerSim;

    private final VoltageOut voltageOut;

    private final StatusSignal<Voltage> _rightRollerVoltage;
    
    private final StatusSignal<Voltage> _leftRollerVoltage;

    public IntakeIOSim(final Constants.IntakeConstants intakeConstants) {
        this.intakeConstants = intakeConstants;
        this.deltaTime = new DeltaTime(true);

        this.rightRoller = new TalonFX(intakeConstants.rightRollerMotor());
        this.leftRoller = new TalonFX(intakeConstants.leftRollerMotor());

        this.intakeBeamBreak = new DigitalInput(1);
        this.intakeBeamBrakeSim = new DIOSim(intakeBeamBreak);
        this.intakeBeamBrakeSim.setInitialized(true);
        this.intakeBeamBrakeSim.setIsInput(true);
        this.intakeBeamBrakeSim.setValue(true);

        final DCMotorSim rightRollerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.18121 / (2 * Math.PI),
                        2.9787 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1)
        );

        this.rightRollerSim = new TalonFXSim(
                rightRoller,
                rightRollerMotorSim::update,
                rightRollerMotorSim::setInputVoltage,
                rightRollerMotorSim::getAngularPositionRad,
                rightRollerMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim leftRollerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.19557 / (2 * Math.PI),
                        2.9856 / (2 * Math.PI)
                ),
                DCMotor.getKrakenX60Foc(1)
        );

        this.leftRollerSim = new TalonFXSim(
                leftRoller,
                leftRollerMotorSim::update,
                leftRollerMotorSim::setInputVoltage,
                leftRollerMotorSim::getAngularPositionRad,
                leftRollerMotorSim::getAngularVelocityRadPerSec
        );

        this.voltageOut = new VoltageOut(0);

        this._rightRollerVoltage = rightRoller.getMotorVoltage();

        this._leftRollerVoltage = leftRoller.getMotorVoltage();


        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            rightRollerSim.update(dt);
            leftRollerSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d,%d)",
                rightRoller.getDeviceID(),
                leftRoller.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
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
        final InvertedValue rightInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration();
        rightRollerConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(0.18121)
                .withKA(2.9787)
                .withKP(3.6111);
        rightRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        rightRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        rightRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        rightRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightRollerConfig.MotorOutput.Inverted = rightInvertedValue;
        rightRollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        rightRoller.getConfigurator().apply(rightRollerConfig);

        final InvertedValue leftInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration();
        leftRollerConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKV(0.19557)
                .withKA(2.9856)
                .withKP(1.955);
        leftRollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leftRollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        leftRollerConfig.CurrentLimits.StatorCurrentLimit = 60;
        leftRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftRollerConfig.MotorOutput.Inverted = leftInvertedValue;
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

        SimUtils.setCTRETalonFXSimStateMotorInverted(rightRoller, rightInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(leftRoller, leftInvertedValue);
    }

    @Override
    public void toVoltage(double rightRollerVolts, double leftRollerVolts) {
        rightRoller.setControl(voltageOut.withOutput(rightRollerVolts));
        leftRoller.setControl(voltageOut.withOutput(leftRollerVolts));
    }

    @Override
    public void setBeamBreakSensorState(final boolean intakeBeamBroken) {
        intakeBeamBrakeSim.setValue(!intakeBeamBroken);
    }
}