package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoaderSubsystem extends SubsystemBase{
    private LoaderIO loaderIO;
    private LoaderIOInputsAutoLogged loaderInputs = new LoaderIOInputsAutoLogged();

    public LoaderSubsystem(){
        loaderIO = new LoaderIOTalonFX();
        
    }

    @Override
    public void periodic() {
        loaderIO.updateInputs(loaderInputs);
        Logger.processInputs("Arm/Loader", loaderInputs);

    }

    //Sets the angle of the loader pivot (Currently relative to the arm, should probably make it field relative)
    public void setLoaderPivot(Rotation2d position) {
        loaderIO.setPivotPosition(position);
    }

    //Returns the loader pivot angle relative to the arm
    public Rotation2d getLoaderPivotAngle() {
        return Rotation2d.fromRadians(loaderInputs.pivotPosition);
    }

    //Sets the speed of the loader.
    public void setLoaderSpeed(double speed) {
        loaderIO.setRollerSpeed(speed);
    }
    //Sets the speed of the loader Pivot (SHOULD NOT BE USED)
    public void setLoaderPivotSpeed(double speed){
        loaderIO.setPivotSpeed(speed);
    }
    
    //Returns the speed of the laoder
    public Rotation2d getLoaderSpeed() {
        return Rotation2d.fromRadians(loaderInputs.rollerVelocity);
    }
    //Resets the position of the internal loader pivot encoder. (Usually for zeroing)
    public void setLoaderPivotRotor(Rotation2d pos){
        loaderIO.setPivotRotorPos(pos);
    }

    public void setLoaderPivotkG(double kG){
        loaderIO.setLoaderkG(kG);
    }
    public void setLoaderPivotkS(double kS){
        loaderIO.setLoaderkS(kS);
    }
    public void setLoaderPivotkV(double kV){
        loaderIO.setLoaderkV(kV);
    }
    public void setLoaderPivotkP(double kP){
        loaderIO.setLoaderkP(kP);
    }
    public void setLoaderPivotkD(double kD){
        loaderIO.setLoaderkD(kD);
    }
    public double getLoaderPivotSpeed() {
        return loaderInputs.pivotVelocity;
    }

    public double getLoaderPivotCurrent() {
        return loaderInputs.pivotStatorCurrent;
    }
    public TalonFXConfiguration getLoaderPivotConfig() {
        return loaderIO.getPivotConfig();
    }
    public void setLoaderPivotConfig(TalonFXConfiguration config) {
        loaderIO.setPivotConfig(config);
    }

    public boolean getLoaderSensor(){
        return loaderIO.getLoaderSensor();
    }

    public boolean getShooterSensor(){
        return loaderIO.getShooterSensor();
    }
}
