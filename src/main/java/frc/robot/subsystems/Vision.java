package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;;

public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    public Vision() throws IOException {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        poseEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY, 
            camera, 
            VisionConstants.ROBOT_TO_CAMERA
        );
    }

    public Optional<Transform3d> getTransform() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            System.out.println("No targets\n");
            return Optional.empty();
        }

        PhotonTrackedTarget target = result.getBestTarget();

        /* Traditional distance using formula */
        double dist = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.CAMERA_HEIGHT_METERS, 
            VisionConstants.APRILTAG_HEIGHT_METERS, 
            VisionConstants.CAMERA_PITCH_RADIANS, 
            Units.degreesToRadians(target.getPitch())
        );

        System.out.println("yaw: " + target.getYaw());
        System.out.println("range: " + dist);
        System.out.println("camera to target translation/transform: " + target.getBestCameraToTarget() + "\n");

        return Optional.of(target.getBestCameraToTarget());
    

    }

    public Optional<Trajectory> getTrajectory() {
        TrajectoryConfig config = Constants.Trajectory.CONFIG;
        double coefficient = Constants.Trajectory.COEFFICIENT;
        var res = this.getTransform();
        if (res.isEmpty()) {
            DriverStation.reportWarning("No vision targets in range", false);
            return Optional.empty();
        }

        Transform3d transform = res.get();
        Translation2d end = transform.getTranslation().toTranslation2d().minus(new Translation2d(0.5, 0)).times(coefficient);

        /* Pose2d start, List<Translation2D> pathPoints, Pose2d end, config */
        //TODO: Fix Rotation2d of the end pos
        return Optional.of(TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(end.div(2)),
            new Pose2d(end, new Rotation2d(transform.getRotation().getAngle())),
            config
        ));

        
    }
}
