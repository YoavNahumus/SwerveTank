package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final double JOYSTICK_DEADBAND = 0.15;
    public static final double JOYSTICK_ANGLE_DEADBAND = 0.25;
    public static final double JOYSTICK_IDLE_DEADBAND = 0.3;
    public static final double FALCON_CPR = 2048;
    public static final double DRIVE_RATE_LIMIT = 1.5;

    private Constants() {} // Prevents instantiation

    public static final class ChassisConstants {
        public static final int GYRO_ID = -1;

        public static final double ANGLE_KP = -1;
        public static final double ANGLE_KI = -1;
        public static final double ANGLE_KD = -1;

        public static final double TRANSLATION_KP = -1;
        public static final double TRANSLATION_KI = -1;
        public static final double TRANSLATION_KD = -1;

        public static final double ANGLE_TOLERANCE = 1;
        public static final double TRANSLATION_TOLERANCE = 0.02;

        public static final double TRACK_WIDTH = -1;
        public static final double TRACK_LENGTH = -1;
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2),
            new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2),
            new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2),
            new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2)
        );

        public static final double MAX_SPEED = -1;

        public static final double MAX_DRIVE_SPEED = 4;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
    }

    public static enum SwerveModuleConstants {
        LEFT_FRONT(-1, -1, -1, -1),
        LEFT_BACK(-1, -1, -1, -1),
        RIGHT_FRONT(-1, -1, -1, -1),
        RIGHT_BACK(-1, -1, -1, -1);

        public final int leftID, rightID, encoderID;
        public final double encoderOffset;
        private SwerveModuleConstants(double encoderOffset, int leftID, int rightID, int encoderID) {
            this.leftID = leftID;
            this.rightID = rightID;
            this.encoderID = encoderID;
            this.encoderOffset = encoderOffset;
        }


        public static final double VELOCITY_KP = -1;
        public static final double VELOCITY_KI = -1;
        public static final double VELOCITY_KD = -1;

        public static final double ANGLE_KP = -1;
        public static final double ANGLE_KI = -1;
        public static final double ANGLE_KD = -1;

        public static final double GEAR_RATIO = 6;
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double COUNTES_PER_METER = (FALCON_CPR * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
        
        public static final double VELOCITY_KS = -1;
        public static final double VELOCITY_KV = -1;
        public static final SimpleMotorFeedforward FEEDFORWARD = new SimpleMotorFeedforward(VELOCITY_KS, VELOCITY_KV);
        
        public static final double MAX_VELOCITY = (1 - VELOCITY_KS) / VELOCITY_KV;

        public static final double TRACK_WIDTH = -1;
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
    }
}
