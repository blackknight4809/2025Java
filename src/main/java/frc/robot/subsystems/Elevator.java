package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    // Motor controllers.
    private final SparkMax primaryMotor;
    private final SparkMax followerMotor;

    // Encoder and Spark Max built-in PID controller.
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    // Bottom limit switch (wired to DIO2; normally open so that when pressed, bottomLimit.get() returns true).
    private final DigitalInput bottomLimitSwitch;

    // Joystick (port 1) for manual control.
    private final Joystick joystick;

    // Slew rate limiter for smoothing manual input.
    private final SlewRateLimiter slewLimiter = new SlewRateLimiter(0.5);

    // Control state.
    private boolean isHomed = false;
    private double setpoint; // Target position in inches.
    private boolean presetActive = false;

    // Deadband threshold for manual control.
    private static final double DEADBAND = 0.1;

    public Elevator(Joystick joystick) {
        this.joystick = joystick;

        // Instantiate SparkMax controllers using CAN IDs from Constants.
        primaryMotor = new SparkMax(Constants.Elevator.liftLeft, MotorType.kBrushless);
        followerMotor = new SparkMax(Constants.Elevator.liftRight, MotorType.kBrushless);

        // Manually set inversions.
        primaryMotor.setInverted(false);
        followerMotor.setInverted(true);
        // If supported, you can command the follower to follow:
       // followerMotor.follow(primaryMotor);

        // Retrieve encoder and PID controller from the primary.
        encoder = primaryMotor.getEncoder();
        pidController = primaryMotor.getClosedLoopController();

        // Reset encoder initially.
        encoder.setPosition(0.0);

        // Instantiate bottom limit switch on DIO2.
        bottomLimitSwitch = new DigitalInput(Constants.Elevator.limitSwitchPort);

        // Initialize state.
        isHomed = false;
        setpoint = Constants.Elevator.bottomPos;
        presetActive = false;

        // Add SmartDashboard control to reset the encoder.
        SmartDashboard.putData("Reset Elevator Encoder", 
                new InstantCommand(() -> {
                    encoder.setPosition(0);
                }, this));

        // Set the default command.
        // When manual input (joystick Y) is active, we bypass PID and drive open-loop.
        // When no manual input is present, we command the PID to hold the setpoint.
        setDefaultCommand(new RunCommand(() -> {
            // First, check the limit switch. If it's pressed, home the elevator.
            if (isLimitSwitchPressed()) {
                handleBottomLimit();
            }

            // Read manual input from joystick Y (adjust sign as needed; here, positive means upward).
            double manualInput = joystick.getY();

            // Apply deadband.
            manualInput = Math.abs(manualInput) < DEADBAND ? 0.0 : manualInput;
            // Optionally, apply slew rate limiting for smoother control.
            manualInput = slewLimiter.calculate(manualInput);

            if (Math.abs(manualInput) > 0.0) {
                // Manual control: override preset.
                presetActive = false;
                primaryMotor.set(manualInput);
            } else if (presetActive && isHomed) {
                // Preset mode: command the setpoint using built-in PID, with gravity compensation (kG).
                pidController.setReference(inchesToEncoderCounts(setpoint), ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.Elevator.kG);
            } else if (isHomed) {
                // If there's no manual input and no preset, hold the current position.
                double holdPosition = encoder.getPosition();
                pidController.setReference(holdPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.Elevator.kG);
            } else {
                // Not homed, don't drive.
                primaryMotor.set(0.0);
            }
            
            // Update telemetry.
            SmartDashboard.putNumber("Elevator Height (in)", encoderCountsToInches(encoder.getPosition()));
            SmartDashboard.putNumber("Elevator Setpoint (in)", setpoint);
            SmartDashboard.putBoolean("Elevator Homed", isHomed);
            SmartDashboard.putBoolean("Bottom Limit", isLimitSwitchPressed());
        }, this));
    }

    // Utility conversion: inches to encoder counts.
    private double inchesToEncoderCounts(double inches) {
        return inches * Constants.Elevator.countsPerInch;
    }

    // Utility conversion: encoder counts to inches.
    private double encoderCountsToInches(double counts) {
        return counts / Constants.Elevator.countsPerInch;
    }

    // Returns true if the bottom limit switch is pressed.
    private boolean isLimitSwitchPressed() {
        return bottomLimitSwitch.get(); // Wired normally open: returns true when pressed.
    }

    // Homing routine: when bottom limit is pressed, reset encoder and mark elevator as homed.
    private void handleBottomLimit() {
        encoder.setPosition(Constants.Elevator.bottomPos * Constants.Elevator.countsPerInch);
        isHomed = true;
        setpoint = Constants.Elevator.bottomPos;
    }

    /**
     * Sets the elevator target position in inches and activates preset mode.
     * @param inches Target position in inches.
     */
    public void setPositionInches(double inches) {
        if (!isHomed && inches > Constants.Elevator.bottomPos) {
            SmartDashboard.putString("Elevator Warning", "Elevator not homed!");
            return;
        }
        setpoint = MathUtil.clamp(inches, Constants.Elevator.bottomPos, Constants.Elevator.maxPos);
        presetActive = true;
        pidController.setReference(inchesToEncoderCounts(setpoint), ControlType.kPosition, ClosedLoopSlot.kSlot0, Constants.Elevator.kG);
        SmartDashboard.putNumber("Elevator Preset Target (in)", setpoint);
    }

    /**
     * Returns the current elevator height in inches.
     */
    public double getHeightInches() {
        return encoderCountsToInches(encoder.getPosition());
    }
    
    @Override
    public void periodic() {
        // Periodic telemetry is handled in the default command; additional periodic tasks can be added here if needed.
    }
}
