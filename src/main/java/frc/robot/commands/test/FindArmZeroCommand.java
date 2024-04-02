package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class FindArmZeroCommand extends Command {

    private ArmSubsystem arm = RobotContainer.arm;

    public FindArmZeroCommand() {
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Voltage", 0);
    }

    @Override
    public void execute() {
        double voltage = SmartDashboard.getNumber("Voltage", 0);

        arm.getPivotMotor().setVoltage(voltage);

        SmartDashboard.putNumber("Pos", arm.getArmPivotAngle().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        arm.getPivotMotor().setVoltage(0);
    }

    
}
