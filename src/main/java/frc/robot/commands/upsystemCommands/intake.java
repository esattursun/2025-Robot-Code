// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.upsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.catcher_left;
import frc.robot.subsystems.catcher_right;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intake extends Command {
  /** Creates a new scoreCommand. */
  private final catcher_left CATCHER_LEFT; 
  private final catcher_right CATCHER_RIGHT; 

  
  public intake( catcher_left CATCHER_LEFT, catcher_right CATCHER_RIGHT) {
    this.CATCHER_LEFT= CATCHER_LEFT;  
    this.CATCHER_RIGHT = CATCHER_RIGHT;
    addRequirements(CATCHER_LEFT,CATCHER_RIGHT);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CATCHER_LEFT.setmotor(-0.65);
    CATCHER_RIGHT.setmotor(0.65);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    CATCHER_LEFT.setmotor(0);
    CATCHER_RIGHT.setmotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
