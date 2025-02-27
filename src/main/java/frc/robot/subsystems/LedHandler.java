package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedHandler extends SubsystemBase {

    private final CANdle candle = new CANdle(33);

    private final StrobeAnimation clawStrobe;
    private final StrobeAnimation intakeStrobe;

    public LedHandler() {
        super();

        candle.configFactoryDefault();

        candle.configStatusLedState(false);

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.v5Enabled = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.2;

        clawStrobe = new StrobeAnimation(0, 255, 0, 0, 0.2, 200, 10);
        intakeStrobe = new StrobeAnimation(255, 0, 255, 0, 0.2, 200, 10);

        candle.configAllSettings(configAll, 100);
        candle.setLEDs(255, 0, 255, 0, 8, 400);
    }

    public void setColor(int r, int g, int b){
        candle.setLEDs(r, g, b);
    }

    public void setBlue(){
        candle.setLEDs(0, 0, 255);
    }

    public void clearAnimation(){
        candle.clearAnimation(0);
    }
    
    public void setIntakeStrobe(){
        candle.animate(intakeStrobe);
    }
    public void setClawStrobe(){
        candle.animate(clawStrobe);
    }
}
