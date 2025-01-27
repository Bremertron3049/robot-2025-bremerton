package frc.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.vision.VisionConstants.CameraInfo;

import java.util.List;
import java.util.Optional;

public class Camera {
    private PhotonCamera currentCamera;
    private VisionConstants.CameraInfo cameraInfo;
    private AprilTagFieldLayout fieldLayout;

    public List<PhotonPipelineResult> latestListBuffer = null;


    public Camera(CameraInfo cameraInfo){
        this.cameraInfo = cameraInfo;
        this.currentCamera = new PhotonCamera(cameraInfo.name);
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        if(!currentCamera.isConnected()){
            DriverStation.reportWarning("CAMERA "+cameraInfo.name+" IS NOT CONNECTED!", true);
        }
    }

    //Has two behaviors. Pulls all unread results. If contains unread results, returns them.
    //If there are no unread results, returns the last unread result list that did.
    //This ensures there will always be a list of results returned.
    public List<PhotonPipelineResult> getAllUnreadResults(){
        List<PhotonPipelineResult> results = currentCamera.getAllUnreadResults();
        latestListBuffer = (results.size() != 0) ? results : latestListBuffer;
        return latestListBuffer;
    }

    //Given a PhotonPipelineResult, calculates the most accurate Pose3d. robotHeading used for more accurate tracking.
    public Pose3d processSingleTarget(PhotonPipelineResult result, Optional<Double> robotHeading){
        PhotonTrackedTarget target = null;
        Pose3d targetPose = null;

        // Sets target to the tag with the lowest pose ambiguty.
        double lowest = 1;
        for(PhotonTrackedTarget tag : result.targets){
            Optional<Pose3d> pose = fieldLayout.getTagPose(tag.fiducialId);
            if(tag.poseAmbiguity < lowest && pose.isPresent()){
                  lowest = tag.poseAmbiguity;
                  target = tag;
                  targetPose = pose.get();
            }
        }

        if(target == null || targetPose == null) return null;

        Transform3d bestTransform3d = target.getBestCameraToTarget();
        Transform3d altTransform3d = target.getAlternateCameraToTarget();

        Pose3d bestPose3d = PhotonUtils.estimateFieldToRobotAprilTag(
            bestTransform3d,
            targetPose,
            cameraInfo.offset.inverse()
        );
        Pose3d altPose3d = PhotonUtils.estimateFieldToRobotAprilTag(
            altTransform3d,
            targetPose,
            cameraInfo.offset.inverse()
        );

        if(robotHeading.isPresent()){
            Pose3d temp;
            double bestH = bestPose3d.getRotation().getZ();
            double altH = altPose3d.getRotation().getZ();

            if(Math.abs(robotHeading.get() - bestH) > Math.abs(robotHeading.get() - altH)){
                temp = bestPose3d;
                bestPose3d = altPose3d;
                altPose3d = temp;
            }
        }

        return bestPose3d;
    }

    public Pose3d processMultiTarget(PhotonPipelineResult result, Optional<Pose3d> robotPose){
        if(result.getMultiTagResult().isEmpty()){
            return null;
        }

        MultiTargetPNPResult targets = result.getMultiTagResult().get();

        Transform3d bestTransform3d = targets.estimatedPose.best;
        Transform3d altTransform3d = targets.estimatedPose.alt;

        // This is minimal error-checking, implement cross-comparison with robotPose.
        bestTransform3d = (targets.estimatedPose.bestReprojErr > targets.estimatedPose.altReprojErr) ? altTransform3d : bestTransform3d;

        return new Pose3d(
            bestTransform3d.getTranslation(),
            bestTransform3d.getRotation()
        ).plus(cameraInfo.offset.inverse());
    }
}
