package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swervedrive.SwerveConstants;
import frc.robot.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

// OdometryI supplies a single instance (INSTANCE) of Odometry and is used to distribute it across code,
// as well as providing any high level functions.

public class OdometryI extends SubsystemBase{
    private static SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    

    // The Odometry instance.
    private static SwerveDriveOdometry INSTANCE = new SwerveDriveOdometry(
        SwerveConstants.KINEMATICS,
        swerveSubsystem.getRotation2d(),
        swerveSubsystem.getModulePositions()
    );

    private static SwerveDriveOdometry TOTAL_INSTANCE = INSTANCE;

    @Override
    public void periodic(){
        INSTANCE.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions());
        TOTAL_INSTANCE.update(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions());
    }
    
    // Returns the instance if initialized, and an empty SwerveDriveOdometry if not.
    public static SwerveDriveOdometry getInstance(){
        if(INSTANCE == null){
            DriverStation.reportError("OdometryI instance has not been initialized.", true);
            return null;
        }

        return INSTANCE;
    }

    public static SwerveDriveOdometry getTotalInstance(){
        if(TOTAL_INSTANCE == null){
            DriverStation.reportError("OdometryI Total instance has not been initialized.", true);
            return null;
        }

        return TOTAL_INSTANCE;
    }

    public static Pose3d convert(Pose3d fieldPose){
        Pose3d pose = new Pose3d(INSTANCE.getPoseMeters());
        return fieldPose.plus(new Transform3d(pose, pose));
    }

    public static void zero(){
        INSTANCE.resetPose(Pose2d.kZero);
    }

    /*
     * Pose can be reset using INSTANCE.resetPose(pose)
     */

    // COMMENTED OUT IN FAVOR OF STATIC INITIALIZATION
    // Initializes the OdometryI instance. If already set, returns current instance.
    /*public static SwerveDriveOdometry setInstance(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){

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
    }*/
}
