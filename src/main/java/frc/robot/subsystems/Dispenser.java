package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class Dispenser extends SubsystemBase {
  // Carriage motor (brushed) controlled by SparkMax.
  private final SparkMax carriageMotor;
  // External encoder wired to the RoboRIO (assumed channels 0 and 1; adjust as needed).
  private final Encoder carriageEncoder;
  // PID controller for closed-loop carriage control.
  private final PIDController carriagePID;
  // Flag and target for preset (closed-loop) mode.
  private boolean presetMode = false;
  private double carriageTarget = 0.0;

  // Shooter motor (brushed, no encoder).
  private final SparkMax shooterMotor;

  // Joystick for carriage control (generic gamepad on controller port 1).
  private final Joystick dispenserJoystick;

  // Preset positions for carriage (encoder units).
  // Far Left = 0, Center = 1138, Far Right = 2236.
  private static final double PRESET_LEFT   = 0.0;
  private static final double PRESET_CENTER = 1138.0;
  private static final double PRESET_RIGHT  = 2236.0;

  // Deadband for manual control.
  private static final double DEADBAND = 0.15;
  
  // Define encoder limits.
  private static final double MIN_ENCODER_LIMIT = 0.0;
  private static final double MAX_ENCODER_LIMIT = 2236.0;

  public Dispenser(Joystick joystick) {
    // Instantiate the carriage motor using its CAN ID from Constants.
    carriageMotor = new SparkMax(Constants.Dispenser.carriage, MotorType.kBrushed);
    // Instantiate the shooter motor.
    shooterMotor  = new SparkMax(Constants.Dispenser.shooter, MotorType.kBrushed);

    // Instantiate the external encoder.
    carriageEncoder = new Encoder(0, 1);
    carriageEncoder.setDistancePerPulse(1.0);

    // Instantiate a PIDController for carriage closed-loop control.
    carriagePID = new PIDController(0.1, 0.0, 0.0);
    carriagePID.setTolerance(5.0);

    // Instantiate the generic gamepad (Joystick) on port 1.
    dispenserJoystick = joystick;
    
    // Add a SmartDashboard widget to allow manual resetting of the carriage encoder.
    SmartDashboard.putData("Reset Carriage Encoder", 
        new InstantCommand(() -> carriageEncoder.reset(), this));

    // Set the default command to handle manual carriage control with preset override.
    setDefaultCommand(new RunCommand(() -> {
      // Get the raw manual input from the joystick's X axis.
      double manualInput = dispenserJoystick.getX();
      
      // Cancel preset mode if any significant manual input is detected.
      if (Math.abs(manualInput) > 0.05) {
          presetMode = false;
      }
      
      // Apply deadband.
      if (Math.abs(manualInput) < DEADBAND) {
          manualInput = 0.0;
      }
      
      // Limit maximum speed to 0.5.
      double maxSpeed = 0.5;
      manualInput = MathUtil.clamp(manualInput, -maxSpeed, maxSpeed);
      
      // Enforce encoder limits on manual input.
      double currentEncoder = carriageEncoder.getDistance();
      if (currentEncoder >= MAX_ENCODER_LIMIT && manualInput < 0) {
           manualInput = 0.0;
      }
      if (currentEncoder <= MIN_ENCODER_LIMIT && manualInput > 0) {
           manualInput = 0.0;
      }
      
      if (Math.abs(manualInput) > 0.0) {
          carriageMotor.set(manualInput);
      } else if (presetMode) {
          double output = carriagePID.calculate(currentEncoder, carriageTarget);
          carriageMotor.set(output);
      } else {
          carriageMotor.set(0.0);
      }
    }, this));
  }

  @Override
  public void periodic() {
    // Update the SmartDashboard with the current carriage encoder value.
    SmartDashboard.putNumber("Carriage Encoder", carriageEncoder.getDistance());
  }

  /**
   * Commands the carriage to move to a specified target position using PID control.
   *
   * @param target The target encoder position.
   */
  public void goToCarriagePosition(double target) {
    // Clamp the target to within the safe range.
    carriageTarget = MathUtil.clamp(target, MIN_ENCODER_LIMIT, MAX_ENCODER_LIMIT);
    presetMode = true;
    carriagePID.reset();
    carriageMotor.set(carriagePID.calculate(carriageEncoder.getDistance(), carriageTarget));
  }

  // Convenience methods for preset carriage positions.
  public void goToLeft() {
    goToCarriagePosition(PRESET_LEFT);
  }

  public void goToCenter() {
    goToCarriagePosition(PRESET_CENTER);
  }

  public void goToRight() {
    goToCarriagePosition(PRESET_RIGHT);
  }

  /**
   * Sets the shooter motor speed.
   * If the input speed is 0, the motor stops; otherwise, it runs at full speed (1.0).
   *
   * @param speed Motor output value (ignored if nonzero, set to 1.0; 0 stops the motor).
   */
  public void setShooterSpeed(double speed) {
    if (speed == 0.0) {
      shooterMotor.set(0.0);
    } else {
      shooterMotor.set(1.0);
    }
  }
}
