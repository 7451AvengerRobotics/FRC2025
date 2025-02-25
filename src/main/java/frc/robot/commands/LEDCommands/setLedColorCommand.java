package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedHandler;

public class setLedColorCommand extends Command {

    private final LedHandler led;
    private final int rVal;
    private final int gVal;
    private final int bVal;

    public setLedColorCommand(LedHandler led, int rVal, int gVal,int bVal ) {
        this.led = led;
        this.rVal = rVal;
        this.gVal = gVal;
        this.bVal = bVal;

        addRequirements(led);
    }

    @Override
    public void initialize() {
        led.clearAnimation();
    }

    @Override
    public void execute() {
        led.setColor(rVal, gVal, bVal);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    
}
