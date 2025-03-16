package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.AlgeaRemover;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator;
  private final Dispenser dispenser;
  private final AlgeaRemover algeaRemover;

  // Controllers:
  // Operator joystick on port 1 will control Elevator and Dispenser.
  private final Joystick operatorJoystick = new Joystick(1);
  // Xbox controller on port 0 is used for driving and AlgeaRemover.
  private final XboxController driveController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Instantiate subsystems.
    // Elevator and Dispenser now use the operator joystick (port 1) for manual control.
    elevator = new Elevator(operatorJoystick);
    dispenser = new Dispenser(operatorJoystick);
    // AlgeaRemover uses the Xbox controller (port 0).
    algeaRemover = new AlgeaRemover(driveController);
    
    // Configure all button bindings.
    configureBindings();
  }

  private void configureBindings() {
    // --- Elevator Preset Bindings ---
    // Bound to the operator joystick.
    JoystickButton elevatorBottomButton = new JoystickButton(operatorJoystick, 2);
    JoystickButton elevatorLowButton    = new JoystickButton(operatorJoystick, 5);
    JoystickButton elevatorMidButton    = new JoystickButton(operatorJoystick, 8);
    JoystickButton elevatorHighButton   = new JoystickButton(operatorJoystick, 11);
    
    elevatorBottomButton.onTrue(new InstantCommand(() -> elevator.goToBottom(), elevator));
    elevatorLowButton.onTrue(new InstantCommand(() -> elevator.goToLow(), elevator));
    elevatorMidButton.onTrue(new InstantCommand(() -> elevator.goToMid(), elevator));
    elevatorHighButton.onTrue(new InstantCommand(() -> elevator.goToHigh(), elevator));
    
    // --- Dispenser Preset Bindings ---
    // Bound to the operator joystick.
    JoystickButton dispenserRightButton  = new JoystickButton(operatorJoystick, 14);
    JoystickButton dispenserCenterButton = new JoystickButton(operatorJoystick, 16);
    JoystickButton dispenserLeftButton   = new JoystickButton(operatorJoystick, 13);
    
    dispenserRightButton.onTrue(new InstantCommand(() -> dispenser.goToRight(), dispenser));
    dispenserCenterButton.onTrue(new InstantCommand(() -> dispenser.goToCenter(), dispenser));
    dispenserLeftButton.onTrue(new InstantCommand(() -> dispenser.goToLeft(), dispenser));
    
    // --- Dispenser Shooter Motor Bindings ---
    // Bound to the Xbox controller so the shooter motor runs only while the button is held.
    JoystickButton shooterForwardButton = new JoystickButton(driveController, 6);
    JoystickButton shooterBackwardButton = new JoystickButton(driveController, 5);
    
    shooterForwardButton.whileTrue(new RunCommand(() -> dispenser.setShooterSpeed(0.5), dispenser));
    shooterBackwardButton.whileTrue(new RunCommand(() -> dispenser.setShooterSpeed(-0.5), dispenser));
    shooterForwardButton.onFalse(new InstantCommand(() -> dispenser.setShooterSpeed(0.0), dispenser));
    shooterBackwardButton.onFalse(new InstantCommand(() -> dispenser.setShooterSpeed(0.0), dispenser));
    
    // --- AlgeaRemover Preset Bindings ---
    // Bound to the Xbox controller.
    JoystickButton algeaRemoverPreset1Button = new JoystickButton(driveController, 7);
    JoystickButton algeaRemoverPreset2Button = new JoystickButton(driveController, 8);
    
    algeaRemoverPreset1Button.onTrue(algeaRemover.lowAlgea());
    algeaRemoverPreset2Button.onTrue(algeaRemover.highAlgea());
  }

  public Command getAutonomousCommand() {
    // Return your autonomous command here.
    return null;
  }

  public Joystick getOperatorJoystick() {
    return operatorJoystick;
}
}
