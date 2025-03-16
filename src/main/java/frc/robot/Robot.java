// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.studica.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private AHRS navX;
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  
  public Robot() {
    // Instantiate RobotContainer. This sets up subsystems and button bindings.
    m_robotContainer = new RobotContainer();
  }
  
  @Override
  public void robotInit() {
    // Initialize navX on the MXP port.
    navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    navX.reset();
    
    // Start capturing video from the USB camera in a try-catch.
    try {
      UsbCamera camera = CameraServer.startAutomaticCapture("USB Camera", 0);
      // Use a low resolution and modest FPS to reduce bandwidth demands.
      //camera.setResolution(160, 240);
      //camera.setFPS(15);
      // No fancy brightness or pixel format settings—keep it basic.
    } catch (Exception e) {
      System.err.println("Camera initialization error: " + e.getMessage());
    }
  }
  
  @Override
  public void robotPeriodic() {
    // Run the command scheduler.
    CommandScheduler.getInstance().run();
    
    // Update SmartDashboard with navX data.
    SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
    SmartDashboard.putNumber("NavX Angle", navX.getAngle());
    
    // Log operator joystick button states.
 //   for (int i = 1; i <= 12; i++) {
 //     SmartDashboard.putBoolean("Joystick(1) Button " + i, m_robotContainer.getOperatorJoystick().getRawButton(i));
 //   }
  }
  
  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  
  @Override
  public void teleopPeriodic() {}
  
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  
  @Override
  public void testPeriodic() {}
  
  @Override
  public void simulationInit() {}
  
  @Override
  public void simulationPeriodic() {}
}
