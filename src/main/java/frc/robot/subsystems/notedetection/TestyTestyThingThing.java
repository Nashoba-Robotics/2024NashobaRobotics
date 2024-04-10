package frc.robot.subsystems.notedetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class TestyTestyThingThing {

    public static void main(String[] args){
        Transform2d boop = new Transform2d(1, 1, Rotation2d.fromDegrees(30));

        Pose2d beep = new Pose2d(801983002, 69, new Rotation2d());

        Pose2d newBeep = beep.plus(boop);   
        System.out.println(newBeep.getX());
        System.out.println(newBeep.getY());
    }
    

    
}
