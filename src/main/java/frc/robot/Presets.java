package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Presets {

    public static class Arm {

        //TODO: Shooter pos = -35 deg
        public static final Rotation2d PODIUM_SHOOTER_POS = Rotation2d.fromDegrees(-35);
        public static boolean OVERRIDE_AUTOMATIC_AIM = false;


        public static final Rotation2d AMP_POS = Rotation2d.fromDegrees(-54); //60 or 35
        public static final Rotation2d INTAKE_POS = Rotation2d.fromDegrees(-54);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromDegrees(-54);
        // public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static Rotation2d SPEAKER_OFFSET = Rotation2d.fromDegrees(0); //Operator Input
        public static final Rotation2d SOURCE_POS = Rotation2d.fromDegrees(42);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromDegrees(4);



        public static Rotation2d SPEAKER_SPEED = Rotation2d.fromRadians(375);
        public static final Rotation2d SPEAKER_SPEED_PREP = Rotation2d.fromRadians(200);
        public static final double SPEAKER_PERCENT = 0.9;
        public static final Rotation2d AMP_SPEED = Rotation2d.fromRadians(70.0);

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(2);
    }

    public static final class Intake {
        public static final double INTAKE_SPEED = 0.9;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

    public static final class Loader {
        public static final Rotation2d AMP_POS = Rotation2d.fromDegrees(0);
        public static final Rotation2d INTAKE_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromDegrees(18);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromDegrees(2);

        public static final double AMP_SPEED = -0.3;
        public static final double INTAKE_SPEED = -0.3;
        public static final double SHOOT_SPEED = -1;
        public static final double SOURCE_SPEED = -0.3;
        public static final double TO_SHOOTER_TRANSITION = -0.3;
        public static final double TO_LOADER_TRANSITION = 0.3;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

}
