package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Rotation2d;

// OdometryI supplies a single instance (INSTANCE) of Odometry and is used to distribute it across code.

public class OdometryI {

    // The Odometry instance.
    private static SwerveDriveOdometry INSTANCE;

    private OdometryI(){}
    
    // Returns the instance if initialized, and an empty SwerveDriveOdometry if not.
    public static SwerveDriveOdometry getInstance(){
        if(INSTANCE == null){
            DriverStation.reportWarning("OdometryI instance has not been set.", true);
            return null;
        }

        return INSTANCE;
    }

    // Initializes the OdometryI instance. If already set, returns current instance.
    public static SwerveDriveOdometry setInstance(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){

        if(INSTANCE != null){
            DriverStation.reportWarning("OdometryI instance already set.", true);
            return INSTANCE;
        }

        INSTANCE = new SwerveDriveOdometry(
            kinematics,
            gyroAngle,
            modulePositions
        );

        return INSTANCE;
    }
}
