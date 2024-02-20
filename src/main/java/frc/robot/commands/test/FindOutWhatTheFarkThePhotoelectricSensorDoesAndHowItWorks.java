package frc.robot.commands.test;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class FindOutWhatTheFarkThePhotoelectricSensorDoesAndHowItWorks extends Command {
    
    DigitalInput test = new DigitalInput(0);

    @Override
    public void execute() {
        test.get();
    }

    

}
