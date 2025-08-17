// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class catcher_right extends SubsystemBase {

    public final static TalonFX motor1 = new TalonFX(15);

  private final static AnalogInput sharp = new AnalogInput(1);


  public catcher_right() {}

  public void setmotor (double speed){
    motor1.set(speed);
  }
  public void setrightmotor (DoubleSupplier speed){
    motor1.set(speed.getAsDouble() *0.5);
  }
  
  public boolean hascoral(){
    return sharp.getValue()>1000;
    // return false;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("sharp", hascoral());
  } 

}
