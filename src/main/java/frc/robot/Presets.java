package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Presets {

    public static final class Arm {
        public static final Rotation2d AMP_POS = Rotation2d.fromDegrees(35); //60 or 35
        public static final Rotation2d INTAKE_POS = Rotation2d.fromDegrees(-48);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromDegrees(-48);
        // public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromDegrees(28.44);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromDegrees(2);


        public static final Rotation2d SPEAKER_SPEED = Rotation2d.fromRadians(0);
        public static final double SPEAKER_PERCENT = 0.9;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(5);
    }

    public static final class Intake {
        public static final double INTAKE_SPEED = 0.7;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

    public static final class Loader {
        public static final Rotation2d AMP_POS = Rotation2d.fromRadians(0); //TODO 
        public static final Rotation2d INTAKE_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromDegrees(20);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromDegrees(2);

        public static final double AMP_SPEED = 0;
        public static final double INTAKE_SPEED = -0.7;
        public static final double SHOOT_SPEED = -0.9;
        public static final double SOURCE_SPEED = 0.9;

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

}
