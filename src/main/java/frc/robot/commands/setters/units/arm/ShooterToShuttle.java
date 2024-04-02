package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Governor;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToShuttle extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    Rotation2d speed = Rotation2d.fromRadians(Presets.Arm.SHUTTLE_SPEED.getRadians() + 10);
    boolean high;

    public ShooterToShuttle(boolean high){
        this.high = high;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // speed = high ? Presets.Arm.HIGH_SHUTTLE_SPEED : Presets.Arm.LOW_SHUTTLE_SPEED;
        speed = Rotation2d.fromRadians(high ? SmartDashboard.getNumber("High Shuttle Speed", 0) : SmartDashboard.getNumber("Low Shuttle Speed", 0));

        arm.setShooterSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // return arm.getShooterSpeed().getRadians() >= speed.getRadians();
        return (Governor.getDesiredRobotState()==RobotState.SHUTTLE_HIGH || Governor.getDesiredRobotState()==RobotState.SHUTTLE_LOW);
    }
}
