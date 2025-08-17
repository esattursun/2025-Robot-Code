// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class motor extends SubsystemBase {
  /** Creates a new climb. */
  private final SparkMax  motor = new SparkMax(13,MotorType.kBrushless);

  public motor() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setmotor (DoubleSupplier speed){
    motor.set(speed.getAsDouble());
  } 
  
}
