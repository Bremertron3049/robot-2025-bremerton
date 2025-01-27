package frc.robot.swervedrive.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swervedrive.SwerveConstants;
import frc.robot.swervedrive.SwerveSubsystem;

import java.util.function.Supplier;

public class SwerveController extends Command{
    private final SwerveSubsystem SwerveDrive;
    private final SwerveDriveOdometry Odometry;

    private final Supplier<Double> xSupplier, ySupplier, aSupplier;
    private final Supplier<Boolean> fieldOriented, pointOriented;
    private final Supplier<Pose2d> pointSupplier;

    private final SlewRateLimiter speedLimiter;

    //TODO:
    // - Change use of Odometry to a field-oriented method.

    public SwerveController(SwerveSubsystem SwerveDrive, SwerveDriveOdometry Odometry, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> aSupplier, Supplier<Boolean> fieldOriented, Supplier<Boolean> pointOriented, Supplier<Pose2d> pointSupplier){
        this.SwerveDrive = SwerveDrive;
        this.Odometry = Odometry;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.aSupplier = aSupplier;

        this.fieldOriented = fieldOriented;
        this.pointOriented = pointOriented;

        this.pointSupplier = pointSupplier;

        speedLimiter = new SlewRateLimiter(SwerveConstants.ROBOT_MAX_SPEED*1.5);

        addRequirements(SwerveDrive);
    }

    //Returns the input (-1 to 1) required to face the robot to point.
    public double processPointOriented(double angle, Pose2d point){
        Pose2d interim = new Pose2d(point.getX(), Odometry.getPoseMeters().getY(), Rotation2d.kZero);

        if(interim.getX() - point.getX() == 0){
            angle = 0;
        }else{
            angle = Math.atan(
                (interim.getY() - point.getY()) / (interim.getX() - point.getX()) 
            );

            angle = (angle > 1d) ? 1d : (angle < -1d) ? -1d : angle;
            //angle = 2 / (1 + Math.exp(-theta)) - 1;
        }

        return angle;
    }

    @Override
    public void execute(){
        // Get values from suppliers.
        double x = xSupplier.get();
        double y = ySupplier.get();
        double a = aSupplier.get();

        // If pointOriented is true, swap "a" with processed input.
        a = (pointOriented.get()) ? processPointOriented(a, pointSupplier.get()) : a;

        // Slew limit drive motor motion for smooth driving.
        //y = speedLimiter.calculate(y);

        // Scale inputs down by SPEED_LIMITER, then convert to Meters per Second using the robot's max speed.
        x *= SwerveConstants.SPEED_LIMITER * SwerveConstants.ROBOT_MAX_SPEED;
        y *= SwerveConstants.SPEED_LIMITER * SwerveConstants.ROBOT_MAX_SPEED;
        a *= SwerveConstants.SPEED_LIMITER * SwerveConstants.ROBOT_MAX_SPEED_RADIANS;

        // Create ChassisSpeeds for module state conversions.
        // If fieldOriented is true, creates from field-relative constructor instead.
        ChassisSpeeds chassisSpeeds = (fieldOriented.get()) ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, a, SwerveDrive.getRotation2d()) : new ChassisSpeeds(x, y, a);

        // Convert chassisSpeeds to SwerveModuleState(s).
        SwerveModuleState[] moduleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Apply SwerveModuleStates to modules.
        SwerveDrive.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        SwerveDrive.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}