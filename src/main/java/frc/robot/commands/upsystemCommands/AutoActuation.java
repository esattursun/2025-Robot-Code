// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.upsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.util.Actuation.ActuationConstans.upsystemstates;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist.Wriststates;

public class AutoActuation extends Command {
  /** Creates a new upsystem. */
  private final Elevator elevatorsubsystem ;
  private final Wrist wristsubsystem;
  
  private final upsystemstates upsystemstate;

  public AutoActuation(Elevator elevatorsubsystem,Wrist wristsubsystem,upsystemstates upsystemstate) {
    this.elevatorsubsystem = elevatorsubsystem;
    this.wristsubsystem = wristsubsystem;

    this.upsystemstate = upsystemstate;

    addRequirements(elevatorsubsystem,wristsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    
    if(upsystemstate == upsystemstates.source){
      elevatorsubsystem.setState(ElevatorStates.zero);
      if(Math.abs(elevatorsubsystem.getzeroerror()) < 0.06){
        wristsubsystem.setState(Wriststates.source);
        elevatorsubsystem.setState(ElevatorStates.MANUAL);
      }else{
        wristsubsystem.setState(Wriststates.zero);

      }}else if(upsystemstate == upsystemstates.net){
        elevatorsubsystem.setState(ElevatorStates.net);
        if(Math.abs(elevatorsubsystem.getuppesterror()) < 0.2){
          wristsubsystem.setState(Wriststates.net);
          elevatorsubsystem.setState(ElevatorStates.net);

      }else{
        wristsubsystem.setState(Wriststates.zero);
        elevatorsubsystem.setState(ElevatorStates.net);


      }}
      else{
        wristsubsystem.setState(upsystemstate.getwriststate());
        elevatorsubsystem.setState(upsystemstate.getelevatorstate());
      }
  
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
