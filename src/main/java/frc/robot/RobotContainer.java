package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Dispenser;
import frc.robot.subsystems.AlgeaRemover;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator;
  private final Dispenser dispenser;
  private final AlgeaRemover algeaRemover;

  // Controllers: Xbox for driving (port 0) and a generic gamepad for elevator/dispenser (port 1)
  private final XboxController driveController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(1);

  public RobotContainer() {
    // Instantiate subsystems.
    // Elevator and Dispenser use the operator joystick (port 1).
    elevator = new Elevator();
    dispenser = new Dispenser(operatorJoystick);
    // AlgeaRemover now uses the Xbox controller (port 0) so it can read the right Y axis.
    algeaRemover = new AlgeaRemover(driveController);
    
    // Configure all button bindings.
    configureBindings();
  }

  private void configureBindings() {
    // --- Elevator Button Bindings (preset positions) ---
    // Assume buttons 1-4 on the operator joystick control Elevator presets.
    JoystickButton elevatorBottomButton = new JoystickButton(operatorJoystick, 1);
    JoystickButton elevatorLowButton    = new JoystickButton(operatorJoystick, 2);
    JoystickButton elevatorMidButton    = new JoystickButton(operatorJoystick, 3);
    JoystickButton elevatorHighButton   = new JoystickButton(operatorJoystick, 4);
    
    elevatorBottomButton.onTrue(new InstantCommand(() -> elevator.goToBottom(), elevator));
    elevatorLowButton.onTrue(new InstantCommand(() -> elevator.goToLow(), elevator));
    elevatorMidButton.onTrue(new InstantCommand(() -> elevator.goToMid(), elevator));
    elevatorHighButton.onTrue(new InstantCommand(() -> elevator.goToHigh(), elevator));
    
    // --- Dispenser Carriage Preset Bindings ---
    // Assume buttons 7-9 on the operator joystick control Dispenser carriage presets.
    JoystickButton dispenserRightButton  = new JoystickButton(operatorJoystick, 7);
    JoystickButton dispenserCenterButton = new JoystickButton(operatorJoystick, 8);
    JoystickButton dispenserLeftButton   = new JoystickButton(operatorJoystick, 9);
    
    dispenserRightButton.onTrue(new InstantCommand(() -> dispenser.goToRight(), dispenser));
    dispenserCenterButton.onTrue(new InstantCommand(() -> dispenser.goToCenter(), dispenser));
    dispenserLeftButton.onTrue(new InstantCommand(() -> dispenser.goToLeft(), dispenser));
    
    // --- Dispenser Shooter Motor Bindings ---
    // Use the Xbox controller (port 0) for shooter control.
    // For example, button 6 runs the shooter forward at 0.5 and button 5 runs it in reverse at -0.5.
    JoystickButton shooterForwardButton = new JoystickButton(driveController, 6);
    JoystickButton shooterBackwardButton = new JoystickButton(driveController, 5);
    
    shooterForwardButton.onTrue(new InstantCommand(() -> dispenser.setShooterSpeed(0.5), dispenser));
    shooterBackwardButton.onTrue(new InstantCommand(() -> dispenser.setShooterSpeed(-0.5), dispenser));
    
    // --- AlgeaRemover Preset Bindings ---
    // Bind buttons on the Xbox controller (port 0) for AlgeaRemover preset commands.
    // Button 3 runs the low preset and button 4 runs the high preset.
    JoystickButton algeaRemoverPreset1Button = new JoystickButton(driveController, 3);
    JoystickButton algeaRemoverPreset2Button = new JoystickButton(driveController, 4);
    
    algeaRemoverPreset1Button.onTrue(algeaRemover.lowAlgea());
    algeaRemoverPreset2Button.onTrue(algeaRemover.highAlgea());
  }

  public Command getAutonomousCommand() {
    // Return your autonomous command here.
    return null;
  }
}
