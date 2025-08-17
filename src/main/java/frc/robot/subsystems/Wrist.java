// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  public static enum Wriststates {
    NONE,
    zero,
    algea,
    source,
    gripalgae,
    lb2,
    MANUAL,
    net,
    l4,
    stucksource
  }

  private Wriststates state = Wriststates.MANUAL;

  /** Creates a new pivot. */
  public final static TalonFX motor = new TalonFX(13);
    // private final DutyCycleEncoder enc = new DutyCycleEncoder(1);
    // private final static SparkAbsoluteEncoder encoder = catcher_right.motor1.getAbsoluteEncoder();
      private DoubleSupplier motorout;
      private double setpoint = 0;
    
      public Wrist(DoubleSupplier motorout) {
        this.motorout = motorout;
        motor.setNeutralMode(NeutralModeValue.Brake);
      }
      public void manual(DoubleSupplier motorout) {
        double motorOutput = motorout.getAsDouble() * 1;
        if(motorOutput < 0 ){
    
          if(getheight()> 90 ){
            motorOutput = motorOutput * 0.1;
          }else{ 
            motorOutput = motorOutput * 0.14;
     
          }}else{
          if(getheight()> 90 ){
            motorOutput = motorOutput * 0.11;
          }else{ 
            motorOutput = motorOutput * 0.09;
          }}
        motor.set(motorOutput);
      }
    
      public static double getheight(){  
          // double rawAngle = Math.abs(360 - encoder.getPosition());
      double gear = 37.5;
      double startdegree = 202;
      double rawAngle = Math.abs(360 - ((motor.getPosition().getValueAsDouble()/gear) * 360))-   startdegree ;
      // double wrappedAngle = rawAngle - 4;
  
      return 
      rawAngle;
      // (Math.abs(360-(encoder.getPosition()*360)))-144;
    }
  
    public void pid(double setpoint) {
   try (PIDController fastcontroller = new PIDController(Constants.Pivot.kfastPIDPivot.kP, Constants.Pivot.kfastPIDPivot.kI, Constants.Pivot.kfastPIDPivot.kD);
        PIDController slowcontroller = new PIDController(Constants.Pivot.kslowPIDPivot.kP, Constants.Pivot.kslowPIDPivot.kI, Constants.Pivot.kslowPIDPivot.kD)) {
  
    double error = setpoint - getheight();
  
    fastcontroller.setSetpoint(setpoint);
    slowcontroller.setSetpoint(setpoint);
  
    double upSpeed = fastcontroller.calculate(getheight());
    double downSpeed = -slowcontroller.calculate(getheight());
  
    // double slowUpSpeed = -fastcontroller.calculate(getheight())*.4;
    // double slowDownSpeed = -fastcontroller.calculate(getheight())*.4;
  
      if (error > 0) {  
      if(getheight()>90 ){
        motor.set(-upSpeed*0.13);
      }else{
        if(setpoint > 90){
          motor.set(-upSpeed*0.19);
        }else{
          motor.set(-upSpeed);
        }
      }}else {  
        motor.set(downSpeed);
      }
   
    SmartDashboard.putNumber("pivotspeed", upSpeed);
    SmartDashboard.putNumber("Pivot error", error);
      }
  }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("pivot distance", getheight());
        switch (state) {
          case algea:
            setpoint = Constants.Pivot.algeaangle;
            pid(setpoint);
           break;
           case source:
           setpoint = Constants.Pivot.sourceangle;
           pid(setpoint);
           break;
           case zero: 
           setpoint = Constants.Pivot.zeroangle;
           pid(setpoint);
           break;
           case lb2: 
           setpoint = Constants.Pivot.l2b;
           pid(setpoint);
           break;
           case net: 
           setpoint = Constants.Pivot.netangle;
           pid(setpoint);
           break;
          
           case MANUAL:
           manual(this.motorout);
          default:
           break;
        }

      SmartDashboard.putNumber("error", geterror());
      SmartDashboard.putBoolean("elevatorcanfly", elevatorcanfly());
  
    }
  
    public double geterror(){
      return getheight()- setpoint;
    }
    public static boolean elevatorcanfly(){
      return getheight() < 110;
  }

  public boolean haveAlgea(){
    return true;
  }
  public void setState(Wriststates state) {
    this.state = state;
  }

}
