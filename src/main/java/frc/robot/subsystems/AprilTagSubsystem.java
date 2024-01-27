package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    AprilTagFieldLayout aprilTagFieldLayout;// = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

    private static final double CAMERA_HEIGHT_METERS = 0.685;
    private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29);
    public AprilTagSubsystem(){
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load april tag layout");
        }
    }

    public boolean seesAprilTag(){
        return result.hasTargets();
    }


    @Override
    public void periodic() {

        result = camera.getLatestResult();
         if (result.hasTargets()) {
            
            StringBuilder sb = new StringBuilder();
            for( PhotonTrackedTarget target : result.getTargets()) {

                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    double distance = PhotonUtils.calculateDistanceToTargetMeters(
                                    CAMERA_HEIGHT_METERS,
                                    tagPose.get().getZ(),
                                    CAMERA_PITCH_RADIANS,
                                    Units.degreesToRadians(target.getPitch()));
                    sb.append("Target: ").append(target.getFiducialId())
                        .append(", distance: ").append(distance);
                }
            }
            // System.out.println(sb.toString());
        }
    }
}
