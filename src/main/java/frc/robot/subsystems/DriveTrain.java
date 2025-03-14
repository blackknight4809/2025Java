package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// New REVLib 2025 imports:
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // Leader motor controllers
  private final SparkMax leftLeader;
  private final SparkMax rightLeader;

  // Follower motor controllers
  private final SparkMax leftFollower;
  private final SparkMax rightFollower;

  // DifferentialDrive uses only the leader motors.
  private final DifferentialDrive drive;

  // Slew rate limiters to smooth joystick inputs.
  private final SlewRateLimiter forwardLimiter;
  private final SlewRateLimiter rotationLimiter;

  // Xbox controller for drive control (port 0)
  private final XboxController controller;

  @SuppressWarnings("deprecation")
  public DriveTrain() {
    // Instantiate leader motors using CAN IDs from Constants.
    leftLeader = new SparkMax(Constants.DriveConsstants.frontLeft, MotorType.kBrushless);
    rightLeader = new SparkMax(Constants.DriveConsstants.frontRight, MotorType.kBrushless);

    // Instantiate follower motors.
    leftFollower = new SparkMax(Constants.DriveConsstants.backLeft, MotorType.kBrushless);
    rightFollower = new SparkMax(Constants.DriveConsstants.backRight, MotorType.kBrushless);

    // Optionally reset configurations if needed; restoreFactoryDefaults() is not available in SparkMax.

    // Invert left side motors so that the robot drives straight.
    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    // Right side motors remain non-inverted (or set explicitly if needed)
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);

    // Create a configuration object for the follower motors.
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    // For the left follower, set it to follow the left leader.
    followerConfig.follow(leftLeader);
    // You can also set inversion here if needed (redundant if already set).
    followerConfig.inverted(true);
    // Apply the configuration to the left follower.
    leftFollower.configure(followerConfig, null, null);
    // Optionally, you can call leftFollower.burnFlash() if you want to persist it permanently.

    // For the right follower, create a separate configuration (or reuse with adjustments).
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader);
    rightFollowerConfig.inverted(false);
    rightFollower.configure(rightFollowerConfig, null, null);
    // Optionally, rightFollower.burnFlash();

    // Create DifferentialDrive using only the leader motors.
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Initialize slew rate limiters (units: change per second; adjust values as needed).
    forwardLimiter = new SlewRateLimiter(3.0);
    rotationLimiter = new SlewRateLimiter(3.0);

    // Instantiate the Xbox controller on port 0.
    controller = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

    // Set the default command to drive with arcade controls and slew rate limiting.
    setDefaultCommand(
      new RunCommand(() -> {
        double forward = -controller.getLeftY();  // Invert if necessary so forward is positive.
        double rotation = controller.getLeftX();
        drive.arcadeDrive(forwardLimiter.calculate(forward),
                          rotationLimiter.calculate(rotation));
      }, this)
    );
  }

  /**
   * Allows manual arcade drive control with slew rate limiting.
   *
   * @param forward Forward/backward command.
   * @param rotation Rotation command.
   */
  public void arcadeDrive(double forward, double rotation) {
    drive.arcadeDrive(forwardLimiter.calculate(forward),
                      rotationLimiter.calculate(rotation));
  }

  /** Stops the drive motors. */
  public void stop() {
    drive.stopMotor();
  }
}
