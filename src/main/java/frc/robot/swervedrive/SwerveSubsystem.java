package frc.robot.swervedrive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swervedrive.SwerveConstants.ModuleInfo;

public class SwerveSubsystem extends SubsystemBase{
    private static final SwerveSubsystem INSTANCE = new SwerveSubsystem();

    private final Pigeon2 gyro = new Pigeon2(SwerveConstants.GYRO_CANID);

    private final SwerveModule frontLeft = new SwerveModule(
        ModuleInfo.FRONT_LEFT.driveId,
        ModuleInfo.FRONT_LEFT.angleId,
        ModuleInfo.FRONT_LEFT.cancoderId,
        ModuleInfo.FRONT_LEFT.cancoderOffset,
        ModuleInfo.FRONT_LEFT.driveReversed,
        ModuleInfo.FRONT_LEFT.angleReversed,
        ModuleInfo.FRONT_LEFT.cancoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
        ModuleInfo.FRONT_RIGHT.driveId,
        ModuleInfo.FRONT_RIGHT.angleId,
        ModuleInfo.FRONT_RIGHT.cancoderId,
        ModuleInfo.FRONT_RIGHT.cancoderOffset,
        ModuleInfo.FRONT_RIGHT.driveReversed,
        ModuleInfo.FRONT_RIGHT.angleReversed,
        ModuleInfo.FRONT_RIGHT.cancoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        ModuleInfo.BACK_LEFT.driveId,
        ModuleInfo.BACK_LEFT.angleId,
        ModuleInfo.BACK_LEFT.cancoderId,
        ModuleInfo.BACK_LEFT.cancoderOffset,
        ModuleInfo.BACK_LEFT.driveReversed,
        ModuleInfo.BACK_LEFT.angleReversed,
        ModuleInfo.BACK_LEFT.cancoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        ModuleInfo.BACK_RIGHT.driveId,
        ModuleInfo.BACK_RIGHT.angleId,
        ModuleInfo.BACK_RIGHT.cancoderId,
        ModuleInfo.BACK_RIGHT.cancoderOffset,
        ModuleInfo.BACK_RIGHT.driveReversed,
        ModuleInfo.BACK_RIGHT.angleReversed,
        ModuleInfo.BACK_RIGHT.cancoderReversed
    );

    private SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(800);
                zeroGyro();
            }catch(Exception e){}
        }).start();
    }

    //Returns the SwerveDrive instance.
    public static SwerveSubsystem getInstance(){
        return INSTANCE;
    }

    @Override
    public void periodic() {
        // Telemetry goes here.
    }

    //Zeroes the Pigeon2 gyro.
    public void zeroGyro(){
        gyro.reset();
    }

    //UNIT: Degrees
    public double getHeading(){
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
    }

    //Returns Rotation2d of robot heading.
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    //Returns SwerveModulePosition(s) of each module.
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
        };
    }

    //Sets each module to a state given an array of states.
    public void setModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.ROBOT_MAX_SPEED);

        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    //Stops the motors of every module.
    public void stopModules(){
        frontLeft.stopModule();
        frontRight.stopModule();
        backLeft.stopModule();
        backRight.stopModule();
    }
}
