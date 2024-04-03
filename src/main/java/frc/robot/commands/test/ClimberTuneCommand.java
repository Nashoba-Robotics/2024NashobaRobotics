package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsytem;

public class ClimberTuneCommand extends Command{
    ClimberSubsytem climber;
    
    double lastkS = 0.02;
    double lastkV = 1.2;
    double lastkP = 20;
    double lastkD = 0;

    public ClimberTuneCommand(ClimberSubsytem climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Climber Pos", 0);

        SmartDashboard.putNumber("Climber kS", lastkS);
        SmartDashboard.putNumber("Climber kV", lastkV);
        SmartDashboard.putNumber("Climber kP", lastkP);
        SmartDashboard.putNumber("Climber kD", lastkD);

    }

    @Override
    public void execute() {
        Rotation2d pos = Rotation2d.fromRadians(SmartDashboard.getNumber("Climber Pos", 0));
        climber.setClimberPos(pos);

        double kS = SmartDashboard.getNumber("Climber kS", 0);
        if(kS != lastkS){
            climber.setkS(kS);
            lastkS = kS;
        }
        double kV = SmartDashboard.getNumber("Climber kV", 0);
        if(kV != lastkV){
            climber.setkV(kV);
            lastkV = kV;
        }double kP = SmartDashboard.getNumber("Climber kP", 0);
        if(kP != lastkP){
            climber.setkP(kP);
            lastkP = kP;
        }double kD = SmartDashboard.getNumber("Climber kD", 0);
        if(kD != lastkD){
            climber.setkS(kD);
            lastkD = kD;
        }

    }

    @Override
    public void end(boolean interrupted) {
        climber.setClimberSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
