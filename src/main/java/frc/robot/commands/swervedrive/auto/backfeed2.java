// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class backfeed2 extends Command {
  /** Creates a new backfeed. */
  private boolean finish = false;
  private double startTime = 0;
  private final SwerveSubsystem drivebase;

  public backfeed2( SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);

    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    
    finish = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double elapsedTime = Timer.getFPGATimestamp() - startTime;
      if (elapsedTime < 0.30) {
        drivebase.drive(new ChassisSpeeds(-1,0, 0));
      } else {
        drivebase.drive(new ChassisSpeeds(0, 0, 0)); 
        finish = true; 
      } 

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
