package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PNPResult;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    DrivebaseSubsystem drivebaseSubsystem;
    AprilTagFieldLayout aprilTagFieldLayout;

    // private static final double CAMERA_HEIGHT_METERS = 0.685;
    // private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(29);
    private static final double reprojectionErrorThresholdLow = 1.8;
    private static final double reprojectionErrorThresholdHigh = 5.0;
    Transform2d cameraOffsetTransformRed = new Transform2d(-0.41, 0.0, Rotation2d.fromDegrees(0));
    // Transform3d cameraOffsetTransform3dRed = new Transform3d(-0.03, 0, 0, new Rotation3d(0, 0, Units.degreesToRadians(0)));
    Transform2d cameraOffsetTransformBlue = new Transform2d(-0.41, 0.0, Rotation2d.fromDegrees(180));
    // Transform3d cameraOffsetTransform3dBlue = new Transform3d(-0.03, 0, 0, new Rotation3d(0, 0, Units.degreesToRadians(180)));

    private static final int RED_AMP_TAG_ID = 5;
    private static final int BLUE_AMP_TAG_ID = 6;

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

    private Transform2d getSideTranslation(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                return cameraOffsetTransformBlue;
            }
            if (alliance.get() == DriverStation.Alliance.Red) {
                return cameraOffsetTransformRed;
            }
        }
        return cameraOffsetTransformBlue;
    }

    private boolean isSaneMeasurement(PNPResult estimatedPose) {
        if (estimatedPose.bestReprojErr > reprojectionErrorThresholdLow &&
            estimatedPose.bestReprojErr < reprojectionErrorThresholdHigh ) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    return (estimatedPose.best.getX() < 3.0);
                }
                if (alliance.get() == DriverStation.Alliance.Red) {
                    return (estimatedPose.best.getX() > 13.5);
                }
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult  res = camera.getLatestResult();
        
        if (res.hasTargets()) {
            // PhotonTrackedTarget bestTarget = res.getBestTarget();
            double imageCaptureTime = res.getTimestampSeconds();
            var estimatedPose = res.getMultiTagResult().estimatedPose;
            
            if (estimatedPose.isPresent) {
                
                
                // if (++count % 10 == 0){
                //     System.out.println("error: " + estimatedPose.bestReprojErr);
                // }
                if (isSaneMeasurement(estimatedPose)) {
                    Pose2d adjustedPose = new Pose3d(estimatedPose.best.getTranslation(),
                        estimatedPose.best.getRotation()).toPose2d().transformBy(getSideTranslation().inverse());

                    drivebaseSubsystem.addVisionMeasurement(adjustedPose, imageCaptureTime);
                    // System.out.println("adding measurement " + adjustedPose + ", error: " + estimatedPose.bestReprojErr);
                }
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

    public Pose2d getAmpPose() {
        int aprilTagID = 0;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            aprilTagID = (alliance.get() == DriverStation.Alliance.Red) ? RED_AMP_TAG_ID : BLUE_AMP_TAG_ID;
        }

        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagID);
        if (tagPose.isPresent()) {
            Pose2d tagPose2d = tagPose.get().toPose2d();
            tagPose2d = tagPose2d.transformBy(new Transform2d(0,-1.0,new Rotation2d(0)));
            System.out.println("transformed Tag Pose: " + tagPose2d);
            return tagPose2d;
        }
        return null;
    }
}
