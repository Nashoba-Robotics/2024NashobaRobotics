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
    public void setPivot(Rotation2d position) {
        loaderIO.setPivotPosition(position);
    }

    //Returns the loader pivot angle relative to the arm
    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRadians(loaderInputs.pivotPosition);
    }

    //Sets the speed of the loader.
    public void setRollerSpeed(double speed) {
        loaderIO.setRollerSpeed(speed);
    }
    //Sets the speed of the loader Pivot (SHOULD NOT BE USED)
    public void setPivotSpeed(double speed){
        loaderIO.setPivotSpeed(speed);
    }
    
    //Returns the speed of the laoder
    public Rotation2d getLoaderSpeed() {
        return Rotation2d.fromRadians(loaderInputs.rollerVelocity);
    }
    //Resets the position of the internal loader pivot encoder. (Usually for zeroing)
    public void setPivotRotor(Rotation2d pos){
        loaderIO.setPivotRotorPos(pos);
    }

    public void setPivotkG(double kG){
        loaderIO.setLoaderkG(kG);
    }
    public void setPivotkS(double kS){
        loaderIO.setLoaderkS(kS);
    }
    public void setPivotkV(double kV){
        loaderIO.setLoaderkV(kV);
    }
    public void setPivotkP(double kP){
        loaderIO.setLoaderkP(kP);
    }
    public void setPivotkD(double kD){
        loaderIO.setLoaderkD(kD);
    }
    public double getPivotSpeed() {
        return loaderInputs.pivotVelocity;
    }

    public double getPivotCurrent() {
        return loaderInputs.pivotStatorCurrent;
    }
    public TalonFXConfiguration getPivotConfig() {
        return loaderIO.getPivotConfig();
    }
    public void setPivotConfig(TalonFXConfiguration config) {
        loaderIO.setPivotConfig(config);
    }

    public boolean getLoaderSensor(){
        return loaderInputs.loaderSensor;
    }

    public boolean getShooterSensor(){
        return loaderInputs.shooterSensor;
    }
}
