package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmToShuttle extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    private boolean lob = false;
    Rotation2d targetPos;

    public ArmToShuttle(){
        lob = false;
        addRequirements(arm);
    }
    public ArmToShuttle(boolean high){
        lob = high;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Pose2d pose = RobotContainer.drive.getPose();
        switch (DriverStation.getAlliance().orElse(Alliance.Blue)){
            case Blue:
                if(pose.getX() <= Constants.Field.LENGTH/2 + 0.5)
                break;
            case Red:
                break;
        }
        targetPos = lob ? Presets.Arm.HIGH_SHUTTLE_POS : Presets.Arm.LOW_SHUTTLE_POS;
    }

    @Override
    public void execute() {
        arm.setArmPivot(targetPos);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmPivotAngle().getRadians() - targetPos.getRadians()) <= Presets.Arm.POS_TOLERANCE.getRadians();
    }
}
