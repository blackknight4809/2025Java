package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AlgeaRemover extends SubsystemBase {
    // NEO motor controlled by the new REVLib 2025 SparkMax.
    private final SparkMax motor;
    // Servo-controlled linear actuator.
    private final Servo actuator;
    // Xbox controller (from port 0) for manual control of the poker motor.
    private final XboxController controller;
    
    // Preset output values for the NEO motor.
    private static final double PRESET1_OUTPUT = 0.3;  // for button 3
    private static final double PRESET2_OUTPUT = 0.6;  // for button 4
    
    // Servo positions (in degrees) for the linear actuator.
    private static final double ACTUATOR_EXTENDED  = 90.0;
    private static final double ACTUATOR_RETRACTED = 0.0;
    
    // Joystick deadband threshold to reduce drift.
    private static final double DEADBAND = 0.15;
    
    // Encoder limits: Upper limit is 0, lower limit is -123.
    private static final double ENCODER_MAX = 0.0;
    private static final double ENCODER_MIN = -123.0;
    
    /**
     * Constructs the AlgeaRemover subsystem.
     * @param controller The XboxController on port 0 used to control the poker motor.
     */
    public AlgeaRemover(XboxController controller) {
        this.controller = controller;
        
        // Instantiate the SparkMax for the NEO (poker motor) using the CAN ID from Constants.
        motor = new SparkMax(Constants.algeaRemover.poker, MotorType.kBrushless);
        motor.set(0);
        // Reset the encoder to 0 at initialization.
        motor.getEncoder().setPosition(0);
        
        // Instantiate the linear actuator as a Servo on the PWM channel from Constants.
        actuator = new Servo(Constants.algeaRemover.actuator);
        // Initialize the actuator to its retracted position.
        actuator.setAngle(ACTUATOR_RETRACTED);
        
        // Add a SmartDashboard widget to manually reset the encoder.
        SmartDashboard.putData("Reset AlgeaRemover Encoder", 
            new InstantCommand(() -> motor.getEncoder().setPosition(0), this));
        
        // Set the default command so that the poker motor is controlled by the controller's right Y axis,
        // with added deadband and encoder limits.
        setDefaultCommand(new RunCommand(() -> {
            double input = controller.getRightY();
            // Apply deadband to avoid drift:
            if (Math.abs(input) < DEADBAND) {
                input = 0.0;
            }
            // Read the current encoder position:
            double pos = motor.getEncoder().getPosition();
            // If we're at or above the upper limit (0) and trying to increase (input > 0), stop.
            if (pos >= ENCODER_MAX && input > 0) {
                input = 0.0;
            }
            // If we're at or below the lower limit (-123) and trying to decrease (input < 0), stop.
            if (pos <= ENCODER_MIN && input < 0) {
                input = 0.0;
            }
            motor.set(input);
        }, this));
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
        // Update the SmartDashboard with the current NEO encoder position and motor values.
        SmartDashboard.putNumber("AlgeaRemover Encoder", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Poker Motor Output", motor.getAppliedOutput());
        SmartDashboard.putNumber("Poker Motor Command", controller.getRightY());
    }
}
