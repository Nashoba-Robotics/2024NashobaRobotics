package frc.robot.subsystems.sensors;


import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class SensorIOPhotoswitch implements SensorIO{
    DigitalInput shooter1, shooter2, loader, intake;
    public SensorIOPhotoswitch(){
        shooter1 = new DigitalInput(Constants.Sensors.SHOOTER_PORT_1);
        shooter2 = new DigitalInput(Constants.Sensors.SHOOTER_PORT_2);

        loader = new DigitalInput(Constants.Sensors.LOADER_PORT);

        intake = new DigitalInput(Constants.Sensors.INTAKE_PORT);
    }

    public void updateInputs(SensorIOInputs io){
        io.shooter1 = !shooter1.get();
        io.shooter2 = !shooter2.get();

        io.loader = !loader.get();

        io.intake = intake.get();
    }
}
