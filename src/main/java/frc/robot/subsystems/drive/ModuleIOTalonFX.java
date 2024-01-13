package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class ModuleIOTalonFX implements ModuleIO {

    private SwerveModule module;

    private SwerveModulePosition position;
    private SwerveModuleState state;
    
    private TalonFX moveMotor;
    private TalonFX turnMotor;
    private CANcoder encoder;
    
     public ModuleIOTalonFX(SwerveModuleConstants constants, String canbusName) {
        module = new SwerveModule(constants, canbusName);

        position = module.getCachedPosition();// TODO: test latency compensation getPosition()
        state = module.getCurrentState();

        moveMotor = module.getDriveMotor();
        turnMotor = module.getSteerMotor();
        encoder = module.getCANcoder();
    }    

    public void updateInputs(ModuleIOInputs inputs) {
        position = module.getCachedPosition();
        state = module.getCurrentState();

        inputs.movePosition = position.distanceMeters;
        inputs.moveVelocity = module.getDriveMotor().getVelocity().getValueAsDouble() / Constants.Drive.kDriveGearRatio * Constants.TAU * Constants.Drive.WHEEL_RADIUS;
        inputs.moveVoltage = module.getDriveMotor().getMotorVoltage().getValueAsDouble();
        inputs.moveStatorCurrent = moveMotor.getStatorCurrent().getValue();
        inputs.moveSupplyCurrent = moveMotor.getSupplyCurrent().getValue();

        inputs.turnAbsolutePosition = encoder.getAbsolutePosition().getValue() * 360;
        inputs.turnRotorPosition = turnMotor.getRotorPosition().getValue();
        inputs.turnVelocity = turnMotor.getVelocity().getValue();
        inputs.turnVoltage = turnMotor.getSupplyVoltage().getValue();
        inputs.turnStatorCurrent = turnMotor.getStatorCurrent().getValue();
        inputs.turnSupplyCurrent = turnMotor.getSupplyCurrent().getValue();
    }

    public void set(SwerveModuleState state) {
        // TODO: test MotionMagicExpo
        module.apply(state, DriveRequestType.Velocity, SteerRequestType.MotionMagic);
    }

    public SwerveModulePosition getPosition() {
        return position;
    }

    public SwerveModuleState getState() {
        return state;
    }

}
