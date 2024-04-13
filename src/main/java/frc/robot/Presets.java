package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class Presets {

    public static class Arm {

        //TODO: Shooter pos = -35 deg
        public static final Rotation2d PODIUM_SHOOTER_POS = Rotation2d.fromDegrees(-35);
        public static boolean OVERRIDE_AUTOMATIC_AIM = false;


        public static final Rotation2d AMP_POS = Rotation2d.fromDegrees(105); //60 or 35
        public static final Rotation2d INTAKE_POS = Rotation2d.fromRadians(-0.89);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromRadians(-0.89);
        // public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromDegrees(42);
        public static final Rotation2d LOW_SHUTTLE_POS = Rotation2d.fromDegrees(5);
        public static final Rotation2d HIGH_SHUTTLE_POS = Rotation2d.fromRadians(-0.89);

        public static final Rotation2d CLIMB_PREP_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d CLIMB_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d TRAP_POS = Rotation2d.fromRadians(0);


        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromRadians(0.015);



        public static Rotation2d COAST_SPEED = Rotation2d.fromRadians(100);

        public static Rotation2d SPEAKER_SPEED = Rotation2d.fromRadians(375); // 375
        public static Rotation2d SPEAKER_SPEED_CHECK = SPEAKER_SPEED;
        public static Rotation2d IDLE_SHOOT_SPEED = Rotation2d.fromRadians(200);
        public static double IDLE_NO_NOTE = 0.05;
        public static double IDLE_NOTE = 0.2;


        public static final Rotation2d SPEAKER_SPEED_PREP = Rotation2d.fromRadians(200);
        public static final double SPEAKER_PERCENT = 0.9;
        public static final Rotation2d AMP_SPEED = Rotation2d.fromRadians(62.0);
        public static final Rotation2d SHUTTLE_SPEED = Rotation2d.fromRadians(300);
        public static final Rotation2d LOW_SHUTTLE_SPEED = Rotation2d.fromRadians(250);
        public static final Rotation2d HIGH_SHUTTLE_SPEED = Rotation2d.fromRadians(215);

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(2);
    }

    public static final class Climber {
        public static final Rotation2d CLIMB_PREP_POS = Rotation2d.fromRadians(11.5);
        public static final Rotation2d CLIMB_POS = Rotation2d.fromRadians(3);
        public static final Rotation2d TRAP_POS = Rotation2d.fromRadians(0);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromRadians(1.5);

        public static double SERVO_CLIMB_POS = 0;
    }

    public static final class Intake {
        public static final double INTAKE_SPEED = 0.9;
        public static final double SHOOT_SPEED = 0.8;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

    public static final class Loader {
        public static final Rotation2d AMP_POS = Rotation2d.fromDegrees(65);
        public static final Rotation2d INTAKE_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromDegrees(0);

        public static final Rotation2d CLIMB_PREP_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d CLIMB_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d TRAP_POS = Rotation2d.fromRadians(0);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromDegrees(2);

        public static final double AMP_SPEED = -0.9;
        public static final double INTAKE_SPEED = -0.45;
        public static final double SHOOT_SPEED = -1;
        public static final double SOURCE_SPEED = -0.3;
        public static final double SHUTTLE_SPEED = -0.9;
        public static final double TO_SHOOTER_TRANSITION = -0.3;
        public static final double TO_LOADER_TRANSITION = 0.3;
        public static final double TRAP_SPEED = 0;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

}
