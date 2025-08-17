// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.util.Conversions;
import frc.util.PIDConst;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be   used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  
  //public static final double MAX_SPEED  = Units.feetToMeters(10);
  
  public static final double MAX_SPEED  = Units.feetToMeters(10.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.06;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class ElevatorConstants
  {

    public static final double kElevatorKp = 5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;

    public static final double kElevatorkS = 0.0; // volts (V)
    public static final double kElevatorkG = 0.762; // volts (V)
    public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorGearing = 6.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kCarriageMass = 4.0; // kg


    public static final double kRotaionToMeters = kElevatorDrumRadius * 2 * Math.PI;
    public static final double kRPMtoMPS = (kElevatorDrumRadius * 2 * Math.PI) / 60;
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.34705;

    public static final double kElevatorMaxVelocity = 3.5;
    public static final double kElevatorMaxAcceleration = 2.5;
  }

  
  public static final class ElevatorConst {
    
    public static double zeroHeight = 0.02; // 0.14 // 0.22; // 0.21; // 0.13
    public static double L2Height = 0.24; // 0.11;
    public static double L3Height = 0.58  ; // 0.11;
    public static double L4Height = 1.124; // 0.11;
    public static double underalgaeHeight = 0.32; // 0.11;
    public static double upperalgaeHeight = 0.67; // 0.11;
    public static double netheight = 1.135; // 0.11;



    public static final PIDConst kPIDElevator = new PIDConst(2.4 , 0.0, 0.03, 0.0);
    public static final double kElevatorVelocity = 0.35; // 0.25
    public static final double kElevatorMotorMaxOutput = 0.4;
    public static double kElevatorMotorMaxOutputClimb = 0.45; // 0.2;

    
    public static final double kAtGoalTolerance = 0.01;

    public static final double kElevatorWinchDiameterM = Conversions.inchesToMeters(1.5);
    public static final double kElevatorWinchCircumM = kElevatorWinchDiameterM * Math.PI;

    public static final boolean kLeftMotorIsInverted = true; // true
    public static final boolean kRightMotorIsInverted = false; // false

    public static int finishvalue = 0; 

    public static void changefinish(int newFinishValue) {
        finishvalue = newFinishValue; 
    }

    public static int getfinish() {
        return finishvalue; 
    }
} 

public static final class Pivot {
    public static double zeroangle = 46;
    // public static double l4angle = 48;

    public static double algeaangle = 70; 
    public static double sourceangle = 136; 
    public static double l2b = 90; 
    public static double stucksource = 128; 
    public static double netangle = 137; 



    
    public static final PIDConst kfastPIDPivot = new PIDConst(0.015, 0.0, 0.0);
    public static final PIDConst kslowPIDPivot = new PIDConst(0.0034, 0.0, 0.0);
}
public static class TELEOP_AUTO_ALIGN {

        public static final PIDController TRANS_CONTROLLER = new PIDController(
          4,
          0,
          0);

              public static final AngularVelocity TURN_SPEED = edu.wpi.first.units.Units.DegreesPerSecond.of(360);

      public static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
          3, 0, 0, new TrapezoidProfile.Constraints(TURN_SPEED.in(edu.wpi.first.units.Units.DegreesPerSecond),
              Math.pow(TURN_SPEED.in(edu.wpi.first.units.Units.DegreesPerSecond), 2)));

        public static HolonomicDriveController TELEOP_AUTO_ALIGN_CONTROLLER = new HolonomicDriveController(
          TRANS_CONTROLLER,
          TRANS_CONTROLLER,
          ROTATION_CONTROLLER);
    }


}

