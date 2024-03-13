package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class Camera {
    private final PhotonCamera camera;

    private PhotonPoseEstimator poseEstimator;

    public Camera(String name) {
        camera = new PhotonCamera(name);
        poseEstimator = null;

        if (RobotBase.isSimulation()) {
            PhotonCamera.setVersionCheckEnabled(false);
        }
    }

    public Camera(String name, Transform3d robot2Cam) {
        camera = new PhotonCamera(name);

        if (RobotBase.isSimulation()) {
            PhotonCamera.setVersionCheckEnabled(false);
        }

        if (camera.getPipelineIndex() == 0) {
            // Attempt to load the AprilTagFieldLayout that will tell us where
            // the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            // Create pose estimator
            poseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            camera, robot2Cam);
            poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult results = camera.getLatestResult();
        if (results.hasTargets()) {
            return results.getBestTarget();
        }
        return null;
    }

    public Optional<EstimatedRobotPose> getPose(Pose2d prevEstimatedRobotPose) {
        if (poseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }
}
