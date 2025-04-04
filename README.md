# 2025-REV-ION-FRC-Starter-Bot

This project provides starting Java code for the 2025 REV ION FRC Starter Bot.

For the most up-to-date information about the 2025 REV ION FRC Starter Bot, watch the [REV website](https://www.revrobotics.com/ion/frc-starter-bot/) and [Starter Bot Chief Delphi thread](https://www.chiefdelphi.com/t/2025-rev-ion-starterbot-for-reefscape/480005)

## Deploying this project to your robot

Before you can deploy this project to your robot, you'll need to set your team number in `/.wpilib/wpilib_preferences.json` or via the "Set Team Number" tool in WPILib VSCode. You will need to update WPILib VSCode to a 2025+ release to build this project.

## Driving the 2025 REV ION FRC Starter Bot

All controls for the Starter Bot are located in `/src/main/java/frc/robot/RobotContainer.java` and can be configured with WPILib's Command Controller API. The project is configured for an Xbox controller by default, so other controllers may require some additional setup or changes.

The robot's starting/zero configuration is:

- elevator at the bottom
- arm on hard stops
- ball intake out

### Default Controls

| Button | Function |
| --- | --- |
| Left stick | Drive/strafe drivetrain |
| Right stick | Rotate drivetrain |
| Start button | Zero swerve heading |
| Left stick button | Set swerve wheels to an X |
| Left bumper | Run coral intake |
| Right bumper | Reverse coral intake |
| B button | Send elevator/arm to feeder station position |
| A button | Send elevator/arm to Level 1/2 position |
| X button | Send elevator/arm to Level 3 position |
| Y button | Send elevator/arm to Highest position |
| Right trigger | Run algae intake |
| Left trigger | Reverse algae intake |

### Other Controls

- Pressing the `User` button on the RoboRIO will zero the encoders for coral elevator and arm and algae arm

## Other features

### Configuration

All configuration objects are instantiated in `/src/main/java/frc/robot/Configs.java` and called by each subsystem

### MAXMotion

The Elevator and Arm for scoring Coral both implement MAXMotion Position Mode

### Simulation

The Coral and Algae Subsystems include full simulation examples, complete with `Mechanism2d` visualizations. Check out the [REVLib simulation docs](https://docs.revrobotics.com/revlib/spark/sim) for more information on setting up and running a simulation. Select `NetworkTables` > `SmartDashboard` to open the `Mechanism2d` windows in the Simulation GUI. These displays will also update with a robot connected, so play around with them and add them to your dashboard! They can be configured in `/src/main/java/frc/robot/Constants.java`.
