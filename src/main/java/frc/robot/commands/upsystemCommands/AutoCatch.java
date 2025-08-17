// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.upsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.catcher_left;
import frc.robot.subsystems.catcher_right;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCatch extends Command {
  /** Creates a new AutoCatch. */
    private final catcher_right catcher_RIGHT;
    private final catcher_left catcher_LEFT;

    private double speed;

  public AutoCatch(catcher_left catcher_LEFT, catcher_right catcher_RIGHT,double speed) {
    this.catcher_LEFT = catcher_LEFT;
    this.catcher_RIGHT = catcher_RIGHT;
    this.speed = speed;
    addRequirements(catcher_LEFT,catcher_RIGHT);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean frontsensor =catcher_LEFT.frontsensor() ;
    boolean backsensor =catcher_LEFT.backsensor() ;
    if(!frontsensor && !backsensor){
      setmotors(speed);
    }else if(frontsensor && !backsensor){
      Commands.run( ()-> setmotors(-0.7)).withTimeout(0.3).andThen(Commands.run( ()-> setmotors(0)));
    }else{
      setmotors(0);
    }
  }

  private void setmotors(double speeds){
    catcher_LEFT.setmotor(speeds);
    catcher_RIGHT.setmotor(-speeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setmotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
