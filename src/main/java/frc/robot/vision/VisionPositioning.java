package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.util.OdometryI;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
import java.util.Optional;

// Subsystem solely dedicated to broad-stroke position estimation
// using vision processing.

// The idea of this is to have completely Odometry-less position estimation
// in order to rule out possible errors.

public class VisionPositioning extends SubsystemBase{
    private final SwerveSubsystem swerveDrive = SwerveSubsystem.getInstance();
    //private final OdometryI odometry = OdometryI.setInstance(null, null, null)

    private static double robotHeading = 0;
    public static Pose3d robotPose = null;
    
    private List<Pose3d> previousEstimates = null;


    @Override
    public void periodic(){
        // Get heading of robot.
        robotHeading = swerveDrive.getHeading();

        // Create new list for this loop's estimates from each camera.
        List<Pose3d> estimates = List.of();
        
        // Loop through each camera.
        for(PT_Camera camera : Cameras){
            // Get latest camera result.
            PhotonPipelineResult result = camera.camera.getAllUnreadResults().get(0);

            // If no targets are found in latest result, return no position.
            if(!result.hasTargets()) {estimates.add(null); return;}

            // If more than one number of targets, process multi-target. Otherwise, single-target.
            estimates.add(
                (result.multitagResult.isPresent()) ? getMultiTarget(camera, Optional.of(result)) : getSingleTarget(camera, Optional.of(result))
            );
        }

        //If any cameras lack new results, estimate them using robot-relative odometry.
        if(previousEstimates != null){
            for(int i=0;i<Cameras.size();i++){
                if(estimates.get(i) == previousEstimates.get(i)){
                    estimates.set(i, OdometryI.convert(previousEstimates.get(i)));
                }
            }
        }
        OdometryI.zero();
        previousEstimates = estimates;

        Pose3d closest = null;

        // ph my god please clean this up and make more readable
        for(Pose3d pose : estimates){
            if(pose == null) return;
            if(closest == null) {closest = pose; return;}
            if(robotPose == null){
                if((robotHeading - closest.getRotation().getZ()) > (robotHeading - pose.getRotation().getZ())){
                    closest = pose;
                }
            }else{
                Pose3d x1 = closest.relativeTo(robotPose);
                Pose3d x2 = pose.relativeTo(robotPose);
                if(Math.hypot(x1.getX(), x1.getY()) > Math.hypot(x2.getX(), x2.getY())){
                    closest = pose;
                }
                
            }
        }

        robotPose = closest;
        
    }

    private static Pose3d getSingleTarget(PT_Camera camera, Optional<PhotonPipelineResult> resultOpt){
        Camera cc = camera.camera;
        PhotonPipelineResult result;
        if(resultOpt.isEmpty()){
            result = cc.getAllUnreadResults().get(0);
        }
        result = resultOpt.get();

        return cc.processSingleTarget(
            result,
            Optional.of(robotHeading)
        );
    }

    private static Pose3d getMultiTarget(PT_Camera camera, Optional<PhotonPipelineResult> resultOpt){
        Camera cc = camera.camera;
        PhotonPipelineResult result;
        if(resultOpt.isEmpty()){
            result = cc.getAllUnreadResults().get(0);
        }
        result = resultOpt.get();

        

        return cc.processMultiTarget(
            result,
            Optional.of((robotPose == null) ? new Pose3d(0,0,0,new Rotation3d(new Rotation2d(robotHeading))) : robotPose)
        );
    }



    //Ensure to populate with PT_Camera instances as they get added.
    //private static List<PT_Camera> Cameras = List.of(
    //    PT_Camera.DEBUG_CAMERA
    //);

    private static List<PT_Camera> Cameras = List.of(PT_Camera.values());

    public static enum PT_Camera{
        DEBUG_CAMERA(
            new Camera(VisionConstants.CameraInfo.DEBUG_CAMERA)
        );

        public final Camera camera;
        PT_Camera(Camera camera){
            this.camera = camera;
        }
    }
}
