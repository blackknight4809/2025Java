package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;  // Config code commented out
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    // Leader and follower SparkMax controllers.
    private final SparkMax leader;
    private final SparkMax follower;
    
    // Closed-loop controller and encoder from the leader.
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    
    // Joystick (port 1) used for manual control.
    private final Joystick joystick;
    
    // Preset positions (encoder units). Negative values: 0 is bottom, -14 is top.
    private static final double POSITION_BOTTOM = 0.0;
    private static final double POSITION_LOW    = -2.0;
    private static final double POSITION_MID    = -10.0;
    private static final double POSITION_HIGH   = -14.0;
    
    // Deadband for manual control.
    private static final double DEADBAND = 0.2;
    
    // Flag and target for preset control.
    private boolean presetActive = false;
    private double targetPosition = 0.0;
    
    /**
     * Constructs the Elevator subsystem.
     * @param joystick The Joystick on port 1 used for manual control.
     */
    public Elevator(Joystick joystick) {
        this.joystick = joystick;
        
        // Instantiate leader and follower using CAN IDs from Constants.
        leader = new SparkMax(Constants.Elevator.liftLeft, MotorType.kBrushless);
        follower = new SparkMax(Constants.Elevator.liftRight, MotorType.kBrushless);
        
        // -- Configuration code using SparkMaxConfig is commented out --
        /*
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.inverted(false);  // Set leader inversion via config.
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        */
        
        // Manually set leader inversion.
        leader.setInverted(false);
        
        // Retrieve the closed-loop controller and encoder from the leader.
        closedLoopController = leader.getClosedLoopController();
        encoder = leader.getEncoder();
        
        // -- Configuration code for follower is commented out --
        /*
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.inverted(true).follow(leader);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        */
        
        // Manually set follower inversion and command it to follow the leader.
        follower.setInverted(true);
       // follower.follow(leader);
        
        // Set the default command: use the joystick's Y axis for manual control,
        // with deadband, speed limiting, and preset override.
        setDefaultCommand(new RunCommand(() -> {
            double manualInput = joystick.getY();
            // Apply deadband:
            if (Math.abs(manualInput) < DEADBAND) {
                manualInput = 0.0;
            }
            // Limit maximum speed to 0.5:
            double maxSpeed = 0.5;
            manualInput = MathUtil.clamp(manualInput, -maxSpeed, maxSpeed);
    
            if (Math.abs(manualInput) > DEADBAND) {
                presetActive = false;
                leader.set(manualInput);
            } else if (presetActive) {
                closedLoopController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
            } else {
                leader.set(0.0);
            }
        }, this));
        
        // Add a SmartDashboard widget to reset the elevator encoder.
        SmartDashboard.putData("Reset Elevator Encoder", 
            new InstantCommand(() -> encoder.setPosition(0), this));
    }
    
    /**
     * Sets the elevator target position and enables preset mode.
     * @param position The desired encoder setpoint.
     */
    public void goToPosition(double position) {
        targetPosition = position;
        presetActive = true;
        SmartDashboard.putNumber("Elevator Preset Target", position);  // Debug output
        closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
    }
    
    // Convenience methods for preset positions.
    public void goToBottom() { goToPosition(POSITION_BOTTOM); }
    public void goToLow()    { goToPosition(POSITION_LOW); }
    public void goToMid()    { goToPosition(POSITION_MID); }
    public void goToHigh()   { goToPosition(POSITION_HIGH); }
    
    /**
     * Provides manual open-loop control of the elevator.
     * @param speed Motor output (-1.0 to 1.0).
     */
    public void manualControl(double speed) {
        leader.set(speed);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Motor Output", leader.getAppliedOutput());
    }
}
