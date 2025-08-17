// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NetAlignment extends Command {
  /** Creates a new AutoAlignmentCoralStation. */
  private SwerveSubsystem drivebase;
  private double vertical;
  public Command AutoAlignmentReefCommmand,preciseAlignment;
  public 
  NetAlignment(SwerveSubsystem drivebase,double vertical) {
    this.drivebase = drivebase; 
    this.vertical = vertical;
    addRequirements(drivebase);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d targetPose ;

       targetPose =  drivebase.getnetpose(vertical);
       targetPose =new Pose2d( targetPose.getX() ,drivebase.getPose().getY(), targetPose.getRotation());
       
       
    PathConstraints constraints = new PathConstraints(
      drivebase.getSwerveDrive().getMaximumChassisVelocity()-4.5,
      drivebase.getSwerveDrive().getMaximumChassisAngularVelocity()-4.5,
       Units.degreesToRadians(360), Units.degreesToRadians(540));


    AutoAlignmentReefCommmand = AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    AutoAlignmentReefCommmand.initialize();

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.getclosestCoralStation();
    AutoAlignmentReefCommmand.execute();  

    drivebase.changevalue(1);
    // SmartDashboard.putNumber("dist", Math.abs(drivebase.gettrueCoralStationPose(vertical,horizontal).getTranslation().getDistance(drivebase.getPose().getTranslation())));

 }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

    AutoAlignmentReefCommmand.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if(Math.abs(drivebase.gettrueCoralStationPose(vertical,horizontal).getTranslation().getDistance(drivebase.getPose().getTranslation()))<0.3

  if( AutoAlignmentReefCommmand.isFinished()){
      return true;
    }else
    {
      return false;
    }
  }
}
