package frc.robot.commands.setters.units.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ShooterToShootPrep extends Command{
    ArmSubsystem arm = RobotContainer.arm;
    private boolean move = false;

    public  ShooterToShootPrep(){
        this.move = false;
        addRequirements(arm);
    }
    public ShooterToShootPrep(boolean move){
        this.move = move;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED.div(3).times(2));
        if(!move) arm.setShooterSpeed(Presets.Arm.SPEAKER_SPEED);
        else arm.setShooterSpeed(Rotation2d.fromRadians(375));

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
