package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {

    
    
    public enum CameraInfo{
        DEBUG_CAMERA("Arducam_OV9281_USB_Camera", Transform3d.kZero); //new Transform3d(0.0254d, 0.0254d, 0.0254d, Rotation3d.kZero));

        public final String name;
        public final Transform3d offset;
        
        CameraInfo(String name, Transform3d offset){
            this.name = name;
            this.offset = offset;
        }
    }
}
