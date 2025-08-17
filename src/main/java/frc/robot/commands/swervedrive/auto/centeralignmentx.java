// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class centeralignmentx extends SequentialCommandGroup {
;
  /** Creates a new autoalignment. */
  public centeralignmentx(SwerveSubsystem drivebase,Elevator elevatorsubsystem,double vertical ,double horizontal) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutoAlignmentReefAlg(drivebase, vertical, horizontal),
        // new AimatCoralStation(drivebase).withTimeout(0.6),
        new AutoAlignmentReefWithPidAlg(drivebase, vertical, horizontal),
        // new AimatCoralStation(drivebase).withTimeout(0.5),
        // new AutoAlignmentCoralStationWithPid(drivebase, vertical, horizontal),
        Commands.run(()-> drivebase.changevalue(10), drivebase).withTimeout(0.01),
        Commands.run(()-> drivebase.lock(), drivebase).withTimeout(0.2),
        // new LastFeed(drivebase),
        Commands.run(()-> drivebase.changevalue(2), drivebase).withTimeout(0.01)

    );
  }


//   public autoalignment(SwerveSubsystem drivebase,double vertical ,double horizontal,Station station) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new AutoAlignmentReef(drivebase, vertical, horizontal,station),
//       new AutoAlignmentReefWithPid(drivebase, vertical, horizontal,station)
//     );

//   // Use addRequirements() here to declare subsystem dependencies.
// }
}
