package frc.robot.commands.setters.units.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Governor;
import frc.robot.RobotContainer;
import frc.robot.Governor.RobotState;
import frc.robot.subsystems.climber.ClimberSubsytem;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class ClimbToManual extends Command{
    ClimberSubsytem climber = RobotContainer.climber;
    CommandJoystick operatorController = RobotContainer.joysticks.getOperatorController();
    boolean flag;
    double leftClimbPos, rightClimbPos;

    public ClimbToManual(){
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        flag = false;
    }

    @Override
    public void execute() {
        double operatorInput = operatorController.getY() * -0.3;
        
        if(Math.abs(operatorInput) < 0.01){
            if(!flag){
                leftClimbPos = climber.getLeftClimberPos().getRadians();
                rightClimbPos = climber.getRightClibmerPos().getRadians();
            }
            climber.setLeftClimberPos(Rotation2d.fromRadians(leftClimbPos));
            climber.setRightClimberPos(Rotation2d.fromRadians(rightClimbPos));
        }
        else{
            // System.out.println("Mini Ben sux so much");
            climber.setClimberSpeed(operatorInput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        leftClimbPos = climber.getLeftClimberPos().getRadians();
        rightClimbPos = climber.getRightClibmerPos().getRadians();

        climber.setLeftClimberPos(Rotation2d.fromRadians(leftClimbPos));
        climber.setRightClimberPos(Rotation2d.fromRadians(rightClimbPos));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}