package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.apriltags.AprilTagManager;

public class SDFinder extends Command{
    int count = 0;

    int sampleSize = 100;

    double[] xPos;
    double[] yPos;

    double lastX;
    double lastY;

    @Override
    public void initialize() {
        xPos = new double[sampleSize];
        yPos = new double[sampleSize];

        count = 0;
    }

    @Override
    public void execute() {
        if(count < sampleSize){
            Pose3d pos = AprilTagManager.getLeftRobotPos();
            if(AprilTagManager.getLeftAmbiguity() < 0.2){
                xPos[count] = pos.getX();
                yPos[count] = pos.getY();

                lastX = pos.getX();
                lastY = pos.getY();
            }
            else{
                xPos[count] = lastX;    //<-- Not quite corect
                yPos[count] = lastY;
            }
        }

        count ++;
    }

    @Override
    public void end(boolean interrupted) {
        double xMean = 0;
        for(int i = 0; i < sampleSize; i++){
            xMean += xPos[i];
        }
        xMean /= sampleSize;

        double xSD = 0;
        for(int i = 0; i < sampleSize; i++){
            xSD += (xPos[i]-xMean) * (xPos[i]-xMean);
        }
        xSD /= sampleSize;
        xSD = Math.sqrt(xSD);

        SmartDashboard.putNumber("X Mean", xMean);
        SmartDashboard.putNumber("X SD", xSD);

        double yMean = 0;
        for(int i = 0; i < sampleSize; i++){
            yMean += yPos[i];
        }
        yMean /= sampleSize;

        double ySD = 0;
        for(int i = 0; i < sampleSize; i++){
            ySD += (yPos[i]-yMean) * (yPos[i]-yMean);
        }
        ySD /= sampleSize;
        ySD = Math.sqrt(ySD);

        SmartDashboard.putNumber("Y Mean", yMean);
        SmartDashboard.putNumber("Y SD", ySD);

        double dist = Math.sqrt((xMean-8.2255)*(xMean-8.2255) + (yMean-4.1055)*(yMean-4.1055));
        SmartDashboard.putNumber("Dist", dist);
    }

    @Override
    public boolean isFinished() {
        return count >= sampleSize;
    }
}
