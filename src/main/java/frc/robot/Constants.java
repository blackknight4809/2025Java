// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConsstants {
    public static final int backLeft = 4;
    public static final int backRight = 5;
    public static final int frontRight = 3;
    public static final int frontLeft = 2;
  }

  public static class Dispenser {
    public static final int carriage = 6;
    public static final int shooter = 7;
  }

  public static class Elevator {
    public static final int liftLeft = 8;
    public static final int liftRight = 9;
    public static final int limitSwitchPort = 2; // change as needed

    // Elevator positions in inches.
    public static final double bottomPos = 6.0625; // your measured bottom position, if any
    public static final double minPos = 6.0625;      // minimum position (inches)
    public static final double maxPos = 61.125;       // maximum position (inches)
    
    // Encoder conversion.
    public static final double countsPerInch = 20.59 / (61.125 - 6.0625);  // Example: 20.59 counts over the full range.
    
    // PID constants for the elevator.
    public static final double kP = 1.0;  // adjust as needed
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    // Feedforward constants.
    public static final double kS = 0.0;  // static friction term, adjust as needed
    public static final double kG = 0.0;  // gravity term, adjust as needed
    public static final double kV = 0.0;  // velocity term, adjust as needed

    // Maximum output for closed-loop control.
    public static final double max_output = 1.0;

    // Motion profile constraints (in inches per second and inches per second squared).
    public static final double maxVelocity = 10.0;      // adjust as needed
    public static final double maxAcceleration = 20.0;    // adjust as needed
    
    // Position tolerance for PID (in inches).
    public static final double posTolerance = 0.5;       // adjust as needed
  }

  public static class algeaRemover {
    public static final int poker = 10;
    public static final int actuator = 0; // PWM channel for the linear actuator servo
  }
}
