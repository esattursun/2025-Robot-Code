// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.util.Reef.coralStationConstants.Station;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignmentReefWithPidAlg extends Command {
  /** Creates a new AutoAlignmentCoralStation. */
  private SwerveSubsystem drivebase;
  private double vertical;
  private double horizontal;
  private Station station;
  public Command preciseAlignment;
  private double rotationerror;
  private double omega;
  public 
  
  AutoAlignmentReefWithPidAlg(SwerveSubsystem drivebase,double vertical ,double horizontal) {
    this.drivebase = drivebase; 
    this.vertical = vertical;
    this.horizontal = horizontal;
    addRequirements(drivebase);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AutoAlignmentReefWithPidAlg(SwerveSubsystem drivebase,double vertical ,double horizontal,Station station) {
    this.drivebase = drivebase; 
    this.vertical = vertical;
    this.horizontal = horizontal;
    this.station = station;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivebase.getclosestCoralStationx();

     HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
      new PIDController(4.6,0 ,0 ),
      new PIDController(4.6,0 , 0),
      new ProfiledPIDController(35
      ,0 ,0.1 ,new Constraints(5.4,5.4 ))
      );
      Supplier<Pose2d> poseSupplier = () -> drivebase.getPose();

          Supplier<Pose2d> robotPoseSupplier = poseSupplier;
          SwerveSubsystem driveSubsystem = drivebase;
          Pose2d targetPose ;

          if(station != null){
             targetPose = 
            drivebase.gettrueCoralStationPose(station,vertical,horizontal);
          }else {
             targetPose = 
            drivebase.gettrueCoralStationPosex(vertical,horizontal);
          }

      holonomicDriveController.setTolerance(new Pose2d(0.005, 0.005, Rotation2d.fromDegrees(1)));
              ChassisSpeeds speeds = holonomicDriveController.calculate(
    robotPoseSupplier.get(),
    targetPose,
    0,
    targetPose.getRotation()
);
double rotation = drivebase.getclosestCoralStationx().getRotationCoralStation();

omega = drivebase.getTargetSpeeds(0, 0, Rotation2d.fromDegrees(rotation))
                 .omegaRadiansPerSecond * 5;

if (Math.abs(omega) <= 0.1) {
    omega = 0;  
} else {
    omega = Math.max(-1, Math.min(omega, 1));
}

    preciseAlignment = new FunctionalCommand(
                      () -> {},
                      () -> driveSubsystem.drive(new ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        omega
                        )),
                      (interrupted) ->
                              driveSubsystem.drive(new ChassisSpeeds()),
                      holonomicDriveController::atReference
                      );

          preciseAlignment.withTimeout(2).execute();
    drivebase.changevalue(2);

    rotationerror =Math.abs( targetPose.getRotation().getDegrees()-drivebase.getPose().getRotation().getDegrees());

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
    drivebase.drive(new ChassisSpeeds(0, 0, 0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(drivebase.gettrueCoralStationPose(vertical,horizontal).getTranslation().getDistance(drivebase.getPose().getTranslation()))<0.02 
    && rotationerror < 1
    ){
      return true;
    }else
    {
      return false;
    }
  }
}
