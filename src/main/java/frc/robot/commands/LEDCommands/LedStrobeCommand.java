package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedHandler;

public class LedStrobeCommand extends Command {

    private final LedHandler led;
    private boolean strobe;
  

    public LedStrobeCommand(LedHandler led, boolean strobe){
        this.led = led;
        this.strobe = strobe;
   

        addRequirements(led);
    }

    @Override
    public void initialize(){

        led.clearAnimation();
    }

    @Override
    public void execute(){
        if (strobe) {
            led.setIntakeStrobe();
        }else {
            led.setClawStrobe();
        }
    }

    @Override
    public void end(boolean interrupted){
        led.clearAnimation();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    
}