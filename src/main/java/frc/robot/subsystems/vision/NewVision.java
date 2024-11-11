package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;


// Figures out which game pieces are near
public class NewVision extends SubsystemBase{
     
    private Drivetrain drivetrain;
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private Transform3d latestPoseResult;
    //new stuff \/
    private PoseStrategy PoseStrategy;
    private 
    //Forward Camera
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 
    //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

    public NewVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.camera = new PhotonCamera("PhotonVisionCam1");
        this.camera = new PhotonCamera("PhotonVisionCam2"); 
    }

    public void periodic() {
        this.latestResult = this.camera.getLatestResult();
        this.latestPoseResult = this.latestResult.getMultiTagResult().estimatedPose.best;
        // SmartDashboard.putNumber("RobotPositionX", networkTable.);
        
    }8
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public static NewVision getInstance() {
        return instance;
    }

    public PhotonPipelineResult getLatestVisionResult() {
        return latestResult;
    }
        
    

}
