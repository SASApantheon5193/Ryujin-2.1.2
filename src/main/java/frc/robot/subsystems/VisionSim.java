package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.utilities.LimelightHelpers;

public class VisionSim extends SubsystemBase {
    private final AprilTagFieldLayout layout;

    public VisionSim() {
        layout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    }

    @Override
    public void periodic() {
        Pose2d simPose = Robot.driveSubsystem.getPose();
    
        // Find closest visible tags (simulate MegaTag2 behavior)
        var visibleTags = layout.getTags()
            .stream()
            .filter(tag -> simPose.getTranslation().getDistance(tag.pose.getTranslation().toTranslation2d()) < 3.5)
            .limit(3) // Simulate max 3 tags visible
            .map(tag -> new LimelightHelpers.RawFiducial(
                tag.ID,
                tag.pose.getX(),
                tag.pose.getY(),
                tag.pose.getZ(),
                simPose.getTranslation().getDistance(tag.pose.getTranslation().toTranslation2d()),
                2.0, // area
                0.1  // ambiguity
            ))
            .toArray(LimelightHelpers.RawFiducial[]::new);
    
        if (visibleTags.length == 0) return;
    
        LimelightHelpers.PoseEstimate est = new LimelightHelpers.PoseEstimate(
    simPose,
    Timer.getFPGATimestamp(),
    0.01,                            // latency (adjust as needed)
    visibleTags.length,
    1.5,                             // tagSpan (arbitrary for sim)
    2.5,                             // avgTagDist (average simulated distance)
    2.0,                             // avgTagArea
    visibleTags,
    true                             // isMegaTag2
);

    
// Manually convert pose estimate into botpose array and set it
LimelightHelpers.setLimelightNTDoubleArray(
    "limelight",
    "botpose_wpiblue",
    LimelightHelpers.poseEstimateToBotPoseArray(est)
);
        // Push fake botpose for visualization (e.g., NT or AdvantageScope)
        LimelightHelpers.setLimelightNTDoubleArray("limelight", "botpose_orb_wpiblue",
            LimelightHelpers.pose2dToArray(simPose));

        // 👉 3. OPTIONAL DEBUG INFO
        SmartDashboard.putNumber("Simulated Visible AprilTags", visibleTags.length);

    }
}    