package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator;

  // Controllers: Xbox for driving (port 0) and generic gamepad for the elevator (port 1)
  private final XboxController driveController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final Joystick elevatorJoystick = new Joystick(1);

  public RobotContainer() {
    // Instantiate the Elevator subsystem using its default constructor.
    elevator = new Elevator();
    
    // Configure button bindings.
    configureBindings();
  }

  private void configureBindings() {
    // Bind buttons on the elevator gamepad to preset elevator positions.
    JoystickButton bottomButton = new JoystickButton(elevatorJoystick, 1);
    JoystickButton lowButton    = new JoystickButton(elevatorJoystick, 2);
    JoystickButton midButton    = new JoystickButton(elevatorJoystick, 3);
    JoystickButton highButton   = new JoystickButton(elevatorJoystick, 4);
    
    // Use onTrue() with InstantCommand instead of whenPressed().
    bottomButton.onTrue(new InstantCommand(() -> elevator.goToBottom(), elevator));
    lowButton.onTrue(new InstantCommand(() -> elevator.goToLow(), elevator));
    midButton.onTrue(new InstantCommand(() -> elevator.goToMid(), elevator));
    highButton.onTrue(new InstantCommand(() -> elevator.goToHigh(), elevator));
  }

  public Command getAutonomousCommand() {
    // Return your autonomous command here.
    return null;
  }
}