package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Presets {

    public static final class Arm {
        public static final Rotation2d AMP_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d INTAKE_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromRadians(0);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromRadians(0);



        public static final Rotation2d AMP_SPEED = Rotation2d.fromRadians(0);
        public static final Rotation2d SPEAKER_SPEED = Rotation2d.fromRadians(0);

        public static final Rotation2d SPEED_TOLERANCE = Rotation2d.fromRadians(0);
    }

    public static final class Intake {
        public static final double INTAKE_SPEED = 0;

        public static final double SPEED_TOLERANCE = 0;
    }

    public static final class Loader {
        public static final Rotation2d AMP_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d INTAKE_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d NEUTRAL_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SHOOT_POS = Rotation2d.fromRadians(0);
        public static final Rotation2d SOURCE_POS = Rotation2d.fromRadians(0);

        public static final Rotation2d POS_TOLERANCE = Rotation2d.fromRadians(0);

        public static final double AMP_SPEED = 0;
        public static final double INTAKE_SPEED = 0;
        public static final double SHOOT_SPEED = 0;
        public static final double SOURCE_SPEED = 0;

        public static final double SPEED_TOLERANCE = 0;
    }

}
