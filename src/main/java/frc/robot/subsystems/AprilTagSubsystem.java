package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    DrivebaseSubsystem drivebaseSubsystem;
    AprilTagFieldLayout aprilTagFieldLayout;

    // private static final double CAMERA_HEIGHT_METERS = 0.685;
    // private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29);
    Transform2d cameraOffsetTransform = new Transform2d(-0.41, 0.0, Rotation2d.fromDegrees(180));
    Transform3d cameraOffsetTransform3d = new Transform3d(-0.03, 0, 0, new Rotation3d(0, 0, Units.degreesToRadians(180)));

    public AprilTagSubsystem(DrivebaseSubsystem drivebaseSubsystem){
        this.drivebaseSubsystem = drivebaseSubsystem;
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
        PhotonPipelineResult  res = camera.getLatestResult();
        
        if (res.hasTargets()) {
            // PhotonTrackedTarget bestTarget = res.getBestTarget();
            double imageCaptureTime = res.getTimestampSeconds();
            
            
            if (res.getMultiTagResult().estimatedPose.isPresent) {
                Pose2d estimatedPose = new Pose3d(res.getMultiTagResult().estimatedPose.best.getTranslation(),
                    res.getMultiTagResult().estimatedPose.best.getRotation()).toPose2d().transformBy(cameraOffsetTransform.inverse());
                //System.out.println("estimated multitag pose: " + estimatedPose);
                drivebaseSubsystem.addVisionMeasurement(estimatedPose, imageCaptureTime);
            // } else if (bestTarget != null) {
            //     Transform3d camToTargetTrans = bestTarget.getBestCameraToTarget();
            //     //camToTargetTrans.plus( new Transform3d( cameraOffsetTransform.getTranslation(), new Rotation2d());
            //     camToTargetTrans = camToTargetTrans.plus(cameraOffsetTransform3d);
            //     Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
            //     if (tagPose.isPresent()) {
            //         Pose2d pose = tagPose.get().transformBy(camToTargetTrans).toPose2d();
            //         System.out.println("single vision mesaurement from tag " + bestTarget.getFiducialId() + ": " + pose.toString());
            //         drivebaseSubsystem.addVisionMeasurement(pose, imageCaptureTime);
            //     }
            }
        }
    }
}
