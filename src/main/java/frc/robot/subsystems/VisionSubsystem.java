package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  // Latest vision-based robot pose updated using MegaTag2.
  private Pose2d latestVisionPose = new Pose2d();

  // Timestamp (in seconds) for the last vision update.
  private double lastVisionUpdateTime = 0.0;
  
  // Limelight network table name (adjust if you are using a different name).
  private static final String LIMELIGHT_NAME = "limelight";

  @Override
  public void periodic() {
    // Use LimelightHelpers to check for a valid target.
    // getTV returns true if a valid target is detected.
    boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    if (!hasTarget) {
      // No valid target detected, so skip updating the pose.
      return;
    }
    
    // Fetch the vision estimate using the MegaTag2 pipeline.
    // This method looks for the "botpose_orb_wpiblue" data under the hood.
    PoseEstimate visionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
    if (visionEstimate != null && LimelightHelpers.validPoseEstimate(visionEstimate)) {
      // Update our latest vision pose (2D) and its timestamp.
      latestVisionPose = visionEstimate.pose;
      lastVisionUpdateTime = visionEstimate.timestampSeconds;
    }
  }

  /**
   * Returns the most recent vision-based pose estimate.
   * @return The current estimated robot pose from vision.
   */
  public Pose2d getLatestVisionPose() {
    return latestVisionPose;
  }

  /**
   * Returns the timestamp of the last vision update.
   * @return The time (in seconds) at which the vision data was recorded.
   */
  public double getLastVisionUpdateTime() {
    return lastVisionUpdateTime;
  }
}
