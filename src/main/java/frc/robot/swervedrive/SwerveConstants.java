package frc.robot.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {

    // Drive Constants

    public static final double ROBOT_MAX_SPEED = Units.feetToMeters(4.5);
    public static final double ROBOT_MAX_SPEED_RADIANS = (2 * Math.PI) * (ROBOT_MAX_SPEED / 0.319176); // Magic value is WHEEL_CIRCUMFERENCE_METERS.

    public static final double SPEED_LIMITER = 0.4;

    public static final int GYRO_CANID = 17;

    // Kinematic Constants

    public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(26.5);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );


    // Module Constants

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = 0.319176;

    public static final double DRIVE_M_GEAR_RATIO = 1 / 8.14;
    public static final double ANGLE_M_GEAR_RATIO = 1 / 21.43;

    public static final double DRIVE_E_ROT2METER = DRIVE_M_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    public static final double ANGLE_E_ROT2METER = ANGLE_M_GEAR_RATIO * Math.PI * 2;

    public static final double DRIVE_E_RPM2MPS = DRIVE_E_ROT2METER / 60;
    public static final double ANGLE_E_RPM2MPS = ANGLE_E_ROT2METER / 60;

    public static final double DRIVE_KP = 0.0020645;
    public static final double ANGLE_KP = 0.1;

    public enum ModuleInfo {
        FRONT_LEFT(11, 12, 21, Units.degreesToRadians(19.688), true, true, false),
        FRONT_RIGHT(7, 8, 22, Units.degreesToRadians(8.261), true, true, false),
        BACK_LEFT(13, 14, 23, Units.degreesToRadians(235.986), true, true, false),
        BACK_RIGHT(5, 6, 24, Units.degreesToRadians(284.063), true, true, false);


        public final int driveId;
        public final int angleId;
        public final int cancoderId;
        public final double cancoderOffset;
        public final boolean driveReversed;
        public final boolean angleReversed;
        public final boolean cancoderReversed;

        ModuleInfo(int driveId, int angleId, int cancoderId, double cancoderOffset, boolean driveReversed, boolean angleReversed, boolean cancoderReversed){
            this.driveId = driveId;
            this.angleId = angleId;
            this.cancoderId = cancoderId;
            this.cancoderOffset = cancoderOffset;
            this.driveReversed = driveReversed;
            this.angleReversed = angleReversed;
            this.cancoderReversed = cancoderReversed;
        }
    }
}