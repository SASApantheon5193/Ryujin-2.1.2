// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ClimberDownCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  
  private boolean dpadSlowMode = false;

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

      public XboxController getDriverController() {

        return m_driverController.getHID();

    }

      XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

           /* Path follower */
     private final SendableChooser<Command> autoChooser;

     

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("score", m_coralSubSystem.revAutoIntakeCommand());
    NamedCommands.registerCommand("stopscore", m_coralSubSystem.revAutoIntakeStopCommand());
    NamedCommands.registerCommand("armfeeder", m_coralSubSystem.autoArmStart());
    NamedCommands.registerCommand("armscorepos", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
    NamedCommands.registerCommand("clearalgae", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    NamedCommands.registerCommand("l3score", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("climberUpTimed", m_climber.climberUpTimed());
    NamedCommands.registerCommand("twoscorelevel", m_coralSubSystem.setSetpointCommand(Setpoint.kLevelA));
    NamedCommands.registerCommand("slowscore", m_coralSubSystem.reverseIntakeSlow());
    NamedCommands.registerCommand("intake", m_coralSubSystem.AutoIntakeCommand());

    // Configure the button bindings
    configureButtonBindings();
    
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        //autoChooser.addOption("1", new PathPlannerAuto("1"));
        //autoChooser.addOption("2", new PathPlannerAuto("2"));
       // autoChooser.addOption("3", new PathPlannerAuto("3"));
        autoChooser.addOption("1.1", new PathPlannerAuto("1.1"));
        autoChooser.addOption("2.1", new PathPlannerAuto("2.1"));
        autoChooser.addOption("3.1", new PathPlannerAuto("3.1"));
    
      


    // Configure default commands
    {
    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {
            int dpad = getDriverController().getPOV();
            if (dpad != -1) {
                driveWithDPad(dpad);
            } else {
                driveWithJoystick();
            }
        }, m_robotDrive));
    }
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
      //  new RunCommand(
       //     () ->
        //        m_robotDrive.drive(
          //          -MathUtil.applyDeadband(
           //             m_driverController.getLeftY(), OIConstants.kDriveDeadband),
           //         -MathUtil.applyDeadband(
             //           m_driverController.getLeftX(), OIConstants.kDriveDeadband),
             //       -MathUtil.applyDeadband(
             //           m_driverController.getRightX(), OIConstants.kDriveDeadband),
              //      true),
          //  m_robotDrive));
  }

public void robotPeriodic() {
    // Runs the Scheduler
    CommandScheduler.getInstance().run();
    }
    


    private void driveWithDPad(int dpad) {
        boolean reverseDirection = m_coralSubSystem.armCurrentTarget > 30;
    
        double baseSpeed = dpadSlowMode ? 0.1 : 1;  // Adjust slow speed as needed
    
        double xSpeed = 0;
        double ySpeed = 0;
        double rotation = 0;
    
        switch (dpad) {
            case 270:  // Up
                ySpeed = -baseSpeed;
                break;
            case 90:  // Down
                ySpeed = baseSpeed;
                break;
            case 180:  // Right
                xSpeed = baseSpeed;
                break;
            case 0:  // Left
                xSpeed = -baseSpeed;
                break;
            default:
                break;
        }
    
        if (reverseDirection) {
            xSpeed = -xSpeed;
            ySpeed = -ySpeed;
        }
    
        m_robotDrive.drive(xSpeed, ySpeed, rotation, false);
    }
    

private void driveWithJoystick() {
    // Logic for driving with joystick
    double xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
    double rotation = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);

    // Call your robot drive method (ensure it's available for this use)
    m_robotDrive.drive(xSpeed, ySpeed, rotation, true);  // Assuming this drives the robot field-centric
}



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Left Bumper -> Run tube intake
    m_driverController.rightBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    // Right Bumper -> Run tube intake in reverse
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

    m_driverController.rightStick().onTrue(m_climber.climberUpTimed());

    // B Button -> Elevator/Arm to human player position, set ball intake to stow
    // when idle
    m_driverController
        .b()
        .onTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation));

    // A Button -> Elevator/Arm to level 2 position
    m_driverController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    
    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    m_driverController.leftTrigger().whileTrue(new ClimberUpCommand(m_climber));

    m_driverController.rightTrigger().whileTrue(new ClimberDownCommand(m_climber));

    m_driverController.back().onTrue(
        runOnce(() -> {
            dpadSlowMode = !dpadSlowMode;
            SmartDashboard.putBoolean("DPad Slow Mode", dpadSlowMode);
        })
    );
    
    
  }
  

  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return m_coralSubSystem.getSimulationCurrentDraw();
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();

}
}