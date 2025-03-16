package frc.robot;

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
    public static final int limitSwitchPort = 2; // DIO port for bottom limit switch

    // Elevator positions (in inches)
    public static final double bottomPos = 6.0625; // Actual measured bottom position
    public static final double maxPos = 61.125;    // Measured max/top position
    
    // Intermediate stops (adjust these carefully according to your mechanism)
    public static final double L1 = 19.83;  // Low stop
    public static final double L2 = 33.59;  // Mid stop
    public static class L3 { public static final double position = 47.36; }  // High stop

    // Encoder conversion
    public static final double countsPerInch = 20.59 / (maxPos - bottomPos);  

    // PID constants (initial guess, tuning required)
    public static final double kP = 0.8;  // Start around 0.5-1.0
    public static final double kI = 0.0;
    public static final double kD = 0.01;

    // Feedforward constants (especially kG important for gravity compensation)
    public static final double kS = 0.2;  // Static friction (try 0.2-0.4 initially)
    public static final double kG = 0.1;  // Gravity compensation (adjust carefully, start ~0.05-0.15)
    public static final double kV = 0.0;  // Not required if just position control without velocity tracking

    // Motion profile constraints (tune for smoothness)
    public static final double maxVelocity = 10.0;       // Inches per second (adjustable)
    public static final double maxAcceleration = 20.0;   // Inches per second squared

    // PID tolerance for positioning accuracy
    public static final double posTolerance = 0.25; // inches, adjust as needed

    // Max closed-loop output (SparkMax)
    public static final double max_output = 1.0;
  }

  public static class algeaRemover {
    public static final int poker = 10;
    public static final int actuator = 0; // PWM channel for the actuator servo
  }
}
