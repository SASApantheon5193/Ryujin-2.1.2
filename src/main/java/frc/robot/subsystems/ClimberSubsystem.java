package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkFlex climbMotor;

    /**
     * This subsystem controls the climber.
     */
    public ClimberSubsystem() {
        // Set up the climb motor as a brushless motor
        climbMotor = new SparkFlex(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Set CAN timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation.
        climbMotor.setCANTimeout(250);

        // Create and apply configuration for climb motor. Voltage compensation helps
        // the climb behave the same as the battery voltage dips. The current limit helps
        // prevent breaker trips or burning out the motor in the event the climb stalls.
        SparkFlexConfig climbConfig = new SparkFlexConfig();
        climbConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
        climbConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
        climbConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Any periodic updates you want to do for the climber
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed) {
        climbMotor.set(speed);
    }

    /**
     * Command to run the climber up for 2 seconds.
     * 
     * @return The command that runs the climber up for 2 seconds.
     */
    public Command climberUpTimed() {
        return new Command() {

            private final WaitCommand waitCommand = new WaitCommand(5);  // Wait for 2 seconds

            @Override
            public void initialize() {
                // Start the climber moving up
                runClimber(ClimberConstants.CLIMBER_SPEED_UP);
                waitCommand.schedule();
            }

            @Override
            public boolean isFinished() {
                // Command finishes after the wait time (2 seconds)
                return waitCommand.isFinished();
            }

            @Override
            public void end(boolean interrupted) {
                // Stop the climber when the command ends
                runClimber(0);
            }

            @Override
            public void execute() {
                // Execution logic is handled in initialize() and end()
            }
        };
    }
}