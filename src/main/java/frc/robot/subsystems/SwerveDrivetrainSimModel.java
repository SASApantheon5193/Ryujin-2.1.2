package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDrivetrainSimModel {

 // Create simulation state variables
    private final Field2d field = new Field2d();

    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(0.0, new Rotation2d()),
        new SwerveModulePosition(0.0, new Rotation2d()),
        new SwerveModulePosition(0.0, new Rotation2d()),
        new SwerveModulePosition(0.0, new Rotation2d())
    };

    private Pose2d simulatedPose = new Pose2d();

    public void update(ChassisSpeeds speeds, double dtSeconds) {
        double dx = speeds.vxMetersPerSecond * dtSeconds;
        double dy = speeds.vyMetersPerSecond * dtSeconds;
        double dtheta = speeds.omegaRadiansPerSecond * dtSeconds;
    
        var deltaTransform = new Transform2d(dx, dy, new Rotation2d(dtheta));
        simulatedPose = simulatedPose.plus(deltaTransform);
        field.setRobotPose(simulatedPose);
    
        // Approximate drive distance traveled
        double distanceTraveled = Math.hypot(dx, dy);
    
        Rotation2d heading = simulatedPose.getRotation();
    
        for (int i = 0; i < modulePositions.length; i++) {
            double newDistance = modulePositions[i].distanceMeters + distanceTraveled;
            modulePositions[i] = new SwerveModulePosition(newDistance, heading);
        }
    }
    

    public Pose2d getSimulatedPose() {
        return simulatedPose;
    }

    public Field2d getField2d() {
        return field;
    }

    public void reset(Pose2d pose) {
        simulatedPose = pose;
        field.setRobotPose(pose);
    }



    public SwerveModulePosition getModulePosition(int index) {
        return modulePositions[index];
    }
    
    public double getSimulatedDistance() {
        double totalDistance = 0.0;
        for (SwerveModulePosition pos : modulePositions) {
            totalDistance += pos.distanceMeters;
        }
        return totalDistance / modulePositions.length;
    }
    
}