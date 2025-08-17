// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class catcher_left extends SubsystemBase {

    public final static TalonFX motor1 = new TalonFX(14);


  public catcher_left() {}

  public void setmotor (double speed){
    motor1.set(speed);
  }
  public void setleftmotor (DoubleSupplier speed){
    motor1.set(speed.getAsDouble() *0.5);
  }


  @Override
  public void periodic() {
      // This method will be called once per scheduler run
  }
  public boolean frontsensor(){
    return false;
  }

  public boolean backsensor(){
    return false;
  }

  public boolean coralcatched(){
    return false;
  }
  public boolean coraldropped(){
    return false;
  }
}
