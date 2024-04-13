package frc.robot.commands.setters.units.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.climber.ClimberSubsytem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class ClimberToManual extends Command{
    ClimberSubsytem climber = RobotContainer.climber;
    CommandJoystick operatorController = RobotContainer.joysticks.getOperatorController();
    boolean flag;
    Rotation2d climbPos;

    public ClimberToManual(){
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPos(climber.getPos());
        flag = false;
    }

    @Override
    public void execute() {
        double operatorInput = operatorController.getY() * -1;
        
        if(Math.abs(operatorInput) < 0.01 && Math.abs(climber.getSpeed().getRadians()) < 0.1){
            if(!flag){
                climbPos = climber.getPos();
                flag = true;
            }
            climber.setPos(climbPos);
        }
        else{
            climber.setSpeed(operatorInput);
            flag = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbPos = climber.getPos();

        climber.setPos(climbPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
