// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConst;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LastFeed extends Command {
  private boolean finish = false;
  private final SwerveSubsystem drivebase;
  private double startTime = 0;
  private boolean timerStarted = false; // getfinish() == 1 olunca zamanlayıcıyı başlatmak için

  public LastFeed( SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override 
  public void initialize(){
    startTime = 0;
    timerStarted = false;
    finish = false; 
  }
  @Override
  public void execute() {
    // Eğer getfinish() == 1 ve zamanlayıcı başlamamışsa, başlat
    if (ElevatorConst.getfinish() == 1 && !timerStarted) {
      startTime = Timer.getFPGATimestamp();
      timerStarted = true;
    }

    // Eğer zamanlayıcı başladıysa ve 0.6 saniye geçmediyse hareket ettir
    if (timerStarted) {
      double elapsedTime = Timer.getFPGATimestamp() - startTime;
      if (elapsedTime < 0.34) {// 43
        drivebase.drive(new ChassisSpeeds(1, 0, 0));
      } else {
        drivebase.drive(new ChassisSpeeds(0, 0, 0)); // 0.6 sn sonra dur
        finish = true; // Komut tamamlandı
      }
    }
  }

  @Override
  public boolean isFinished() {
    return finish; // Komut bitirme kontrolü
    
  } 

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(0, 0, 0)); // Komut bitince dur
  }
}
