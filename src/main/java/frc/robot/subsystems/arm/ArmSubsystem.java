package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private LoaderIO loaderIO;
    private LoaderIOInputsAutoLogged loaderInputs = new LoaderIOInputsAutoLogged();

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        loaderIO.updateInputs(loaderInputs);
        Logger.processInputs("Arm/Arm", armInputs);
        Logger.processInputs("Arm/Loader", loaderInputs);
    }

    public ArmSubsystem() {
        armIO = new ArmIOTalonFX();
        loaderIO = new LoaderIOTalonFX();
    }

    //Sets the pivot position. (0 should be horizontal to the ground)
    public void setArmPivot(Rotation2d position) {
        armIO.setAngle(position);
    }


    public void setArmPivotRotor(Rotation2d rotorPos){
        armIO.setPivotRotorPos(rotorPos);
    }


    //Returns the arm pivot angle as a Rotation2D

    public Rotation2d getArmPivotAngle() {
        return Rotation2d.fromRadians(armInputs.pivotRotorPosition);
    }

    //Sets the speed of the shooter. Units don't matter b/c Rotation2D
    public void setShooterSpeed(Rotation2d speed) {
        armIO.setShooterSpeed(speed);
    }

    //Returns the speed of the shooter
    public Rotation2d getShooterSpeed() {
        return Rotation2d.fromRadians(armInputs.topShooterSpeed);
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
    public void setLoaderSpeed(Rotation2d speed) {
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
    
    public void setArmPivotSpeed(double speed){
        armIO.setPivotSpeed(speed);
    }
    public void setArmPivotkG(double kG){
        armIO.setPivotkG(kG);
    }
    public void setArmPivotkS(double kS){
        armIO.setPivotkS(kS);
    }
    public void setArmPivotkV(double kV){
        armIO.setPivotkV(kV);
    }
    public void setArmPivotkP(double kP){
        armIO.setPivotkP(kP);
    }
    public void setArmPivotkD(double kD){
        armIO.setPivotkD(kD);
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
}
