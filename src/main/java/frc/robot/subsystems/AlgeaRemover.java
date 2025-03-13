package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AlgeaRemover extends SubsystemBase {
    // NEO motor controlled by the new REVLib 2025 SparkMax.
    private final SparkMax motor;
    // Servo-controlled linear actuator.
    private final Servo actuator;
    
    // Preset output values for the NEO motor.
    private static final double PRESET1_OUTPUT = 0.3;  // for button 3
    private static final double PRESET2_OUTPUT = 0.6;  // for button 4
    
    // Servo positions (in degrees) for the linear actuator.
    private static final double ACTUATOR_EXTENDED  = 90.0;
    private static final double ACTUATOR_RETRACTED = 0.0;
    
    public AlgeaRemover() {
        // Instantiate the SparkMax for the NEO using the CAN ID from Constants.algeaRemover.poker.
        motor = new SparkMax(Constants.algeaRemover.poker, MotorType.kBrushless);
       // motor.restoreFactoryDefaults();
        motor.set(0);
        // Reset the encoder to 0.
        motor.getEncoder().setPosition(0);
        
        // Instantiate the linear actuator as a Servo on the PWM channel from Constants.algeaRemover.actuator.
        actuator = new Servo(Constants.algeaRemover.actuator);
        // Initialize the actuator to its retracted position.
        actuator.setAngle(ACTUATOR_RETRACTED);
    }
    
    /**
     * Returns a command that sets the NEO motor output to PRESET1_OUTPUT,
     * holds for 1 second, then returns the motor output to 0.
     */
    public Command lowAlgea() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> motor.set(PRESET1_OUTPUT), this),
            new WaitCommand(1.0),
            new InstantCommand(() -> motor.set(0), this)
        );
    }
    
    /**
     * Returns a command that sets the NEO motor output to PRESET2_OUTPUT,
     * holds for 1 second, then returns the motor output to 0.
     */
    public Command highAlgea() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> motor.set(PRESET2_OUTPUT), this),
            new WaitCommand(1.0),
            new InstantCommand(() -> motor.set(0), this)
        );
    }
    
    /**
     * Returns a command that deploys the linear actuator.
     * The servo is set to the extended position, held for 1 second, then retracted.
     */
    public Command deployActuatorCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> actuator.setAngle(ACTUATOR_EXTENDED), this),
            new WaitCommand(1.0),
            new InstantCommand(() -> actuator.setAngle(ACTUATOR_RETRACTED), this)
        );
    }
    
    @Override
    public void periodic() {
        // Update the SmartDashboard with the current NEO encoder position.
        SmartDashboard.putNumber("AlgeaRemover Encoder", motor.getEncoder().getPosition());
    }
}