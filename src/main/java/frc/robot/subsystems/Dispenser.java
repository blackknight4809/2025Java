package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  // Preset positions for carriage (encoder units; adjust these values).
  private static final double PRESET_RIGHT  = 0.0;
  private static final double PRESET_CENTER = 500.0;
  private static final double PRESET_LEFT   = 1000.0;

  public Dispenser(Joystick joystick) {
    // Instantiate the carriage motor using its CAN ID from Constants and set it as a brushed motor.
    carriageMotor = new SparkMax(Constants.Dispenser.carriage, MotorType.kBrushed);
    // Instantiate the shooter motor using its CAN ID.
    shooterMotor  = new SparkMax(Constants.Dispenser.shooter, MotorType.kBrushed);

    // Restore factory defaults for consistent behavior.
    //carriageMotor.restoreFactoryDefaults();
    //shooterMotor.restoreFactoryDefaults();

    // Instantiate the external encoder.
    // (Assuming the encoder is connected to DIO channels 0 and 1; change as needed.)
    carriageEncoder = new Encoder(0, 1);
    // Set the distance per pulse (adjust according to your encoder specifications).
    carriageEncoder.setDistancePerPulse(1.0);

    // Instantiate a PIDController for carriage closed-loop control.
    // Example gains; tune these for your mechanism.
    carriagePID = new PIDController(0.1, 0.0, 0.0);
    carriagePID.setTolerance(5.0);

    // Instantiate the generic gamepad (Joystick) on port 1.
    dispenserJoystick = joystick;
  }

  @Override
  public void periodic() {
    // Read manual input from the joystick's X axis.
    double manualInput = dispenserJoystick.getX();

    // If manual input is significant, override preset mode.
    if (Math.abs(manualInput) > 0.1) {
      presetMode = false;
      carriageMotor.set(manualInput);
    } else if (presetMode) {
      // In preset mode, use the PID controller to drive the carriage to the target.
      double currentPosition = carriageEncoder.getDistance();
      double output = carriagePID.calculate(currentPosition, carriageTarget);
      carriageMotor.set(output);
    }
    // Otherwise, if no manual input and not in preset mode, do nothing (or hold current state).
    SmartDashboard.putNumber("Carriage Encoder", carriageEncoder.getDistance());
  }

  /**
   * Commands the carriage to move to a specified target position using PID control.
   *
   * @param target The target encoder position.
   */
  public void goToCarriagePosition(double target) {
    carriageTarget = target;
    presetMode = true;
  }

  // Convenience methods for preset carriage positions.
  public void goToRight() {
    goToCarriagePosition(PRESET_RIGHT);
  }

  public void goToCenter() {
    goToCarriagePosition(PRESET_CENTER);
  }

  public void goToLeft() {
    goToCarriagePosition(PRESET_LEFT);
  }

  /**
   * Sets the shooter motor speed.
   *
   * @param speed Motor output (e.g., 0.5 for forward, -0.5 for reverse).
   */
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  
}