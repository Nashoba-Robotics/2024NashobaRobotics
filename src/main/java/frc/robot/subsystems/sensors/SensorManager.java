package frc.robot.subsystems.sensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorManager extends SubsystemBase{
    private SensorIO io;
    private SensorIOInputsAutoLogged inputs = new SensorIOInputsAutoLogged();

    public SensorManager(){
        io = new SensorIOPhotoswitch();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Sensors", inputs);
    }

    public boolean getShooterSensor(){
        return inputs.shooter1 || inputs.shooter2;
    }

    public boolean getIntakeSensor(){
        return inputs.intake;
    }

    public boolean getLoaderSensor(){
        return inputs.loader;
    }
}
