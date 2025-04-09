// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  private final Field2d m_field = new Field2d();

  private RobotConfig config;

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final SwerveDrivetrainSimModel simModel = new SwerveDrivetrainSimModel();


  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(m_gyro.getAngle()),
    getModulePositions(),
    new Pose2d(),
    MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, Math.toRadians(5.0)),
    MatBuilder.fill(Nat.N3(), Nat.N1(), 0.5, 0.5, Math.toRadians(10))
);


        

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

config = new RobotConfig(
  50.0, // mass in kg (estimated)
  6.0,  // moment of inertia (tweak as needed)
  new ModuleConfig(
    ModuleConstants.kWheelDiameterMeters / 2.0, // radius
    DriveConstants.kMaxSpeedMetersPerSecond,    // max drive speed
    1.19, // coefficient of friction (estimated)
    DCMotor.getNeoVortex(1).withReduction(6.75), // motor config
    40,   // current limit
    1     // motors per module
  ),
  // Module positions (FL, FR, BL, BR)
  new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
  new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
  new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
  new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)
);


    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController(
            new PIDConstants(3.16, 0.0, 0.7),
            new PIDConstants(4.9, 0.0, 0.59)
        ),
        config,
        () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
        this
    );
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      getModulePositions()
  );

  // Vision update using MegaTag2 pose estimate
  PoseEstimate vision = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

  if (LimelightHelpers.validPoseEstimate(vision)
      && vision.tagCount > 0
      && vision.avgTagDist < 3.0
      // && vision.ambiguity < 0.2 // Removed due to missing field
      && Math.abs(getTurnRate()) < 200) {
  
      m_poseEstimator.addVisionMeasurement(vision.pose, vision.timestampSeconds);
  }
  
   // Simulate movement based on commanded speeds
   ChassisSpeeds speeds = getRobotRelativeSpeeds();  // or however you're driving it
   simModel.update(speeds, 0.02); // 20 ms loop

  m_field.setRobotPose(getPose());
  SmartDashboard.putData("Field", m_field);
  SmartDashboard.putString("Estimated Pose", getPose().toString());

  }


  @Override
  public void simulationPeriodic() {
      // Update sim model physics
      ChassisSpeeds speeds = getRobotRelativeSpeeds();
      simModel.update(speeds, 0.02);
  
      Pose2d simPose = simModel.getSimulatedPose();
  
      // Update simulated gyro
      m_gyro.setGyroAngleZ(simPose.getRotation().getDegrees());
  
      // Simulate each module's encoder position
      double simMeters = simModel.getSimulatedDistance(); // You may need to track this in sim model
      Rotation2d heading = simPose.getRotation();
  
      m_frontLeft.simSetPosition(new SwerveModulePosition(simMeters, heading));
      m_frontRight.simSetPosition(new SwerveModulePosition(simMeters, heading));
      m_rearLeft.simSetPosition(new SwerveModulePosition(simMeters, heading));
      m_rearRight.simSetPosition(new SwerveModulePosition(simMeters, heading));
  }
  


  public void updateSmartDashboard() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putString("Fused Pose", getPose().toString());
  }
   
  /**
   * Returns the currently-estimated (fused) pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator (and odometry) to a specific pose.
   *
   * @param pose The pose to reset to.
   */
 
   public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_gyro.getAngle()), getModulePositions(), pose);
}


private SwerveModulePosition[] getModulePositions() {
  return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };
}

  /**
   * Drives the robot using joystick inputs. Speeds are scaled appropriately.
   *
   * @param xSpeed Speed in the x direction.
   * @param ySpeed Speed in the y direction.
   * @param rot Angular speed.
   * @param fieldRelative Whether speeds are field-relative.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(() -> {
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    });
  }

  public Command applyRequest(Runnable request) {
    return new RunCommand(request, this);
  }

  /**
   * Sets the swerve module states.
   *
   * @param desiredStates The desired swerve module states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to a zeroed position. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Returns a command that zeros the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.reset());
  }

  /**
   * Returns the current heading of the robot (normalized to [-180, 180] degrees).
   *
   * @return The robot's heading.
   */
  public double getHeading() {
    double angle = m_gyro.getAngle(IMUAxis.kZ);
    angle = ((angle + 180) % 360 + 360) % 360 - 180; // Normalize to [-180, 180]
    return angle;
  }

  /**
   * Returns the turn rate of the robot, in degrees per second.
   *
   * @return The robot's turn rate.
   */
  public double getTurnRate() {
    double rawRate = m_gyro.getRate(IMUAxis.kZ);
    if (Math.abs(rawRate) < 1.0) return 0.0;
    return rawRate * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

    public void driveRobotRelative(ChassisSpeeds speeds) {
      var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(states[0]);
      m_frontRight.setDesiredState(states[1]);
      m_rearLeft.setDesiredState(states[2]);
      m_rearRight.setDesiredState(states[3]);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
}


  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }
}
