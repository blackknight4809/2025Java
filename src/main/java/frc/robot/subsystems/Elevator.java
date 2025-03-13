package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
    // Leader and follower using the new REVLib 2025 SparkMax.
    private final SparkMax leader;
    private final SparkMax follower;
    
    // Closed-loop controller and encoder from the leader for position control.
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;
    
    // Generic gamepad for manual control and button inputs (controller 1).
    private final Joystick gamepad;
    
    // Preset positions (encoder units; adjust as needed).
    private static final double POSITION_BOTTOM = 0.0;
    private static final double POSITION_LOW    = 500.0;
    private static final double POSITION_MID    = 1000.0;
    private static final double POSITION_HIGH   = 1500.0;
    
    public Elevator() {
        // Instantiate the leader and follower using CAN IDs from Constants.
        leader = new SparkMax(Constants.Elevator.liftLeft, MotorType.kBrushless);
        follower = new SparkMax(Constants.Elevator.liftRight, MotorType.kBrushless);
        
        // Restore factory defaults for predictable behavior.
       // leader.restoreFactoryDefaults();
       // follower.restoreFactoryDefaults();
        
        // Retrieve the closed-loop controller and encoder from the leader.
        closedLoopController = leader.getClosedLoopController();
        encoder = leader.getEncoder();
        
        // Set the leader inversion as needed (assumed non-inverted here).
        leader.setInverted(false);
        
        // Configure the follower using the new configuration system.
        // This sets the follower to follow the leader and inverts its output.
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.inverted(true)    // Invert follower output.
                      .follow(leader);    // Set follower to follow the leader.
        follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // Optionally, call follower.burnFlash() to permanently store these settings.
        
        // Instantiate the generic gamepad on port 1.
        gamepad = new Joystick(1);
        
        // Set the default command for manual open-loop control using the gamepad's Y axis.
        setDefaultCommand(new RunCommand(() -> {
            double manualSpeed = gamepad.getY();  // Use the Y axis from the generic gamepad.
            leader.set(manualSpeed);
        }, this));
    }
    
    /**
     * Commands the elevator to move to a specified target position using closed-loop control.
     *
     * @param position The target encoder position.
     */
    public void goToPosition(double position) {
        closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.0);
    }
    
    // Convenience methods for preset positions (bind these to buttons as desired).
    public void goToBottom() { goToPosition(POSITION_BOTTOM); }
    public void goToLow()    { goToPosition(POSITION_LOW); }
    public void goToMid()    { goToPosition(POSITION_MID); }
    public void goToHigh()   { goToPosition(POSITION_HIGH); }
    
    /**
     * Provides manual open-loop control of the elevator.
     *
     * @param speed Motor output (-1.0 to 1.0).
     */
    public void manualControl(double speed) {
        leader.set(speed);
    }
@Override
public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", leader.getEncoder().getPosition());
    
}
}