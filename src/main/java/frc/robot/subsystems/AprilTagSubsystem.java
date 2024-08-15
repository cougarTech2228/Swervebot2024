package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.google.flatbuffers.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    // DrivebaseSubsystem drivebaseSubsystem;
    AprilTagFieldLayout aprilTagFieldLayout;

    private static final double reprojectionErrorThresholdLow = 1.8;
    private static final double reprojectionErrorThresholdHigh = 5.0;
    Transform2d cameraOffsetTransform = new Transform2d(-0.41, 0.0, Rotation2d.fromDegrees(180));

    private static final int RED_AMP_TAG_ID = 5;
    private static final int BLUE_AMP_TAG_ID = 6;
    private static final int FRONT_LEFT = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_RIGHT = 2;
    private static final int BACK_LEFT = 3;
    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, Math.toRadians(5));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(3));

    ArrayList <String> cameraNames = new ArrayList<>(
        Arrays.asList(
        "AprilTagCamera1",
        "AprilTagCamera2",
        "AprilTagCamera3",
        "AprilTagCamera4"
    ));

    ArrayList <PhotonCamera> cameras = new ArrayList<>(
            Arrays.asList(
            new PhotonCamera(cameraNames.get(FRONT_LEFT)),
            new PhotonCamera(cameraNames.get(FRONT_RIGHT)),
            new PhotonCamera(cameraNames.get(BACK_RIGHT)),
            new PhotonCamera(cameraNames.get(BACK_LEFT))
        ));
    
    double [] lastEstTimestamps = new double[] {
        0,0,0,0
    };
        
    
        
        
        ArrayList <Transform3d> cameraTransforms = new ArrayList<>(
            Arrays.asList(
            //front left
            new  Transform3d(
            -0.2, // x
            -0.07, // y
            0.2286, // z
            new Rotation3d(0, Math.toRadians(-34), Math.toRadians(179))),

            //front right
            new Transform3d(
            -0.2, // x
            -0.07, // y
            0.2286, // z
            new Rotation3d(0, Math.toRadians(-34), Math.toRadians(179))),

            //back right
            new Transform3d(
            -0.2, // x
            -0.07, // y
            0.2286, // z
            new Rotation3d(0, Math.toRadians(-34), Math.toRadians(179))),

            //back left
            new Transform3d(
            -0.2, // x
            -0.07, // y
            0.2286, // z
            new Rotation3d(0, Math.toRadians(-34), Math.toRadians(179)))

        ));
        
        
        ArrayList <PhotonPoseEstimator> poseEstimators;

    Transform2d AMP_TO_CAMERA_TRANSFORM = new Transform2d(0.64,-0.127,new Rotation2d(0));


    public AprilTagSubsystem(DrivebaseSubsystem drivebaseSubsystem){
        // this.drivebaseSubsystem = drivebaseSubsystem;
         
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            System.out.println("loaded april tag layout");
        } catch (IOException e) {
            System.out.println("Failed to load april tag layout");
        }

        poseEstimators = new ArrayList<>(
            Arrays.asList(
            new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                cameras.get(FRONT_LEFT), cameraTransforms.get(FRONT_LEFT)),
            new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                cameras.get(FRONT_RIGHT), cameraTransforms.get(FRONT_RIGHT)),
            new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                 cameras.get(BACK_RIGHT), cameraTransforms.get(BACK_RIGHT)),
            new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                cameras.get(BACK_LEFT), cameraTransforms.get(BACK_LEFT))
            ));

    }

    public boolean seesAprilTag(){
        return result.hasTargets();
    }

    private boolean isSaneMeasurement(PNPResult estimatedPose) {
        // if (estimatedPose.bestReprojErr > reprojectionErrorThresholdLow &&
        //     estimatedPose.bestReprojErr < reprojectionErrorThresholdHigh ) {
        //         return (estimatedPose.best.getX() < 4.0) || (estimatedPose.best.getX() > 10);
        // }
        // return false;
        return (estimatedPose.best.getX() < 4.0) || (estimatedPose.best.getX() > 12);
    }

    @Override
    public void periodic() {
        int index = -1;
        for (PhotonCamera camera : cameras) {
            List<Pose2d> measurements = new ArrayList<>();
            List<Pose3d> targets = new ArrayList<>();
            
            index++;
            boolean connected = camera.isConnected();
            Logger.recordOutput("Vision/" + camera.getName() + "/Connected", connected);
            if (!connected)
                continue;

            PhotonPipelineResult pipelineResult = camera.getLatestResult();
            boolean hasTargets = pipelineResult.hasTargets();
            Logger.recordOutput("Vision/" + camera.getName() + "/HasTargets", hasTargets);
            if (!hasTargets)
                continue;

            List<PhotonTrackedTarget> badTargets = new ArrayList<>();
            for (PhotonTrackedTarget target : pipelineResult.targets) {
                if (target.getPoseAmbiguity() > 0.5) {
                    badTargets.add(target);
                }
            }

            pipelineResult.targets.removeAll(badTargets);
            Logger.recordOutput("Vision/" + camera.getName() + "/badTargets", badTargets.size());

            Optional<EstimatedRobotPose> visionEst = poseEstimators.get(index).update(pipelineResult);

            // if (Robot.isSimulation()) {
            //     double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
            //     boolean newResult = Math.abs(latestTimestamp - lastEstTimestamps[index]) > 1e-5;
            //     rearVisionEst.ifPresentOrElse(
            //             est -> getSimDebugField()
            //                     .getObject("VisionEstimation")
            //                     .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 if (newResult)
            //                     getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });
            //     if (newResult)
            //         lastEstTimestamps[index] = latestTimestamp;
            // }

            boolean posePresent = visionEst.isPresent();
            Logger.recordOutput("Vision/" + camera.getName() + "/HasPose", posePresent);
            if (!posePresent)
                return;

            EstimatedRobotPose estimatedPose = visionEst.get();
            Pose3d pose3d = estimatedPose.estimatedPose;
            var est2dPose = pose3d.toPose2d();

            // *** This line crashes!!! ***
            // Logger.recordOutput("Vision/Rear/Pose", est2dPose);
            Logger.recordOutput("Vision/" + camera.getName() + "/Pose", new double[] {
                    est2dPose.getX(),
                    est2dPose.getY(),
                    est2dPose.getRotation().getRadians()
            });

            Logger.recordOutput("Vision/" + camera.getName() + "/Timestamp", estimatedPose.timestampSeconds);
            Logger.recordOutput("Vision/" + camera.getName() + "/Targets", estimatedPose.targetsUsed.size());
            Logger.recordOutput("Vision/" + camera.getName() + "/Strategy", estimatedPose.strategy);

            // Change our trust in the measurement based on the tags we can see
            // var estStdDevs = getEstimationStdDevs(est2dPose, poseEstimators.get(index), camera);

            // drivebaseSubsystem.addVisionMeasurement(est2dPose, estimatedPose.timestampSeconds, estStdDevs);
            // if (
            //     pose3d.getX() >= -SwerveConstants.VISION_FIELD_MARGIN &&
            //     pose3d.getX() <= Constants.FIELD_LENGTH + SwerveConstants.VISION_FIELD_MARGIN &&
            //     pose3d.getY() >= -SwerveConstants.VISION_FIELD_MARGIN &&
            //     pose3d.getY() <= Constants.FIELD_WIDTH + SwerveConstants.VISION_FIELD_MARGIN &&
            //     pose3d.getZ() >= -SwerveConstants.VISION_Z_MARGIN &&
            //     pose3d.getZ() <= SwerveConstants.VISION_Z_MARGIN
            // ) {
                double sum = 0.0;
                for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                    Optional<Pose3d> tagPose =
                        aprilTagFieldLayout.getTagPose(target.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                    targets.add(tagPose.get());
                    sum += est2dPose.getTranslation().getDistance(tagPose.get().getTranslation().toTranslation2d());
                }

                int tagCount = estimatedPose.targetsUsed.size();
                double stdScale = Math.pow(sum / tagCount, 2.0) / tagCount;
                double xyStd = /*SwerveConstants.VISION_STD_XY_SCALE * */ stdScale;
                double rotStd = /*SwerveConstants.VISION_STD_ROT_SCALE * */ stdScale;

                // drivebaseSubsystem.addVisionMeasurement(est2dPose, estimatedPose.timestampSeconds, VecBuilder.fill(xyStd, xyStd, rotStd));
                measurements.add(est2dPose);
                continue;
            // }
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
            tagPose2d = tagPose2d.transformBy(AMP_TO_CAMERA_TRANSFORM);
            System.out.println("transformed Tag Pose: " + tagPose2d);
            return tagPose2d;
        }
        return null;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedRearGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPoseEstimator estimator, PhotonCamera camera) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    // public Field2d getSimDebugField() {
    //     if (!Robot.isSimulation())
    //         return null;
    //     return visionSim.getDebugField();
    // }
}
