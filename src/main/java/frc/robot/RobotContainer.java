  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
  import edu.wpi.first.math.geometry.Pose2d;
  import edu.wpi.first.math.geometry.Rotation2d;
  import edu.wpi.first.math.kinematics.ChassisSpeeds;
  import edu.wpi.first.wpilibj.DriverStation;
  import edu.wpi.first.wpilibj.Filesystem;
  import edu.wpi.first.wpilibj.RobotBase;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConst;
import frc.robot.Constants.OperatorConstants;
  import frc.robot.commands.swervedrive.auto.AimatCoralStation;
  import frc.robot.commands.swervedrive.auto.NetAlignment;
  import frc.robot.commands.swervedrive.auto.autoalignment;
import frc.robot.commands.swervedrive.auto.backfeed;
import frc.robot.commands.swervedrive.auto.backfeed2;
import frc.robot.commands.swervedrive.auto.centeralignment;
  import frc.robot.commands.swervedrive.auto.centeralignmentx;
  import frc.robot.commands.upsystemCommands.AutoActuation;
  import frc.robot.commands.upsystemCommands.DelayedScore;
  import frc.robot.commands.upsystemCommands.HardScore;
  import frc.robot.commands.upsystemCommands.Score;
import frc.robot.commands.upsystemCommands.intake;
import frc.robot.subsystems.Elevator;
  import frc.robot.subsystems.Elevator.ElevatorStates;
  import frc.robot.subsystems.Wrist;
  import frc.robot.subsystems.Wrist.Wriststates;
  import frc.robot.subsystems.catcher_left;
  import frc.robot.subsystems.catcher_right;
  import frc.robot.subsystems.climb;
  import frc.robot.subsystems.motor;
  import frc.robot.subsystems.swervedrive.SwerveSubsystem;
  import frc.util.Actuation.ActuationConstans.upsystemstates;
  import swervelib.SwerveInputStream;

  /**pP
   * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
   * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
   * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
   */
  public class RobotContainer
  {
    private final motor motor1 = new motor();

      // private final SendableChooser<Command> autoChooser;

    //   public final LEDSubsystem led = new LEDSubsystem();

    public final static CommandXboxController driverXbox = new CommandXboxController(0);
    public final static CommandXboxController driverXbox2 = new CommandXboxController(1);

    public final static catcher_right CATCHER_RIGHT = new catcher_right();
    public final static catcher_left CATCHER_LEFT = new catcher_left();

    public final static Wrist PIVOT = new Wrist(()-> MathUtil.applyDeadband(driverXbox2.getRightY(),0.05));

    public final static Elevator ELEVATOR = new Elevator(()-> driverXbox2.getLeftY());
    
    public final  static climb CLIMB = new climb();



    // Replace with CommandPS4Controller or CommandJoystick if needed

      
    // The robot's subsystems and commands are defined here...
    public static final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                  "swerve/falcon"));

    // private final ElevatorSubsystem elevator = new ElevatorSubsystem();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() *-.8,
                                                                  () -> driverXbox.getLeftX() * -.8)
                                                              .withControllerRotationAxis(()-> 
                                                              -driverXbox.getRightX()*0.6)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

    SwerveInputStream fastdrive = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() *-1.2,
                                                                  () -> driverXbox.getLeftX() * -1.2)
                                                              .withControllerRotationAxis(()-> 
                                                              -driverXbox.getRightX()*0.8)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);
    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                              driverXbox::getRightY)
                                                            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                              .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                          () -> -driverXbox.getLeftY(),
                                                                          () -> -driverXbox.getLeftX())
                                                                      .withControllerRotationAxis(() -> driverXbox.getRightX(
                                                                      ))
                                                                      .deadband(OperatorConstants.DEADBAND)
                                                                      .scaleTranslation(0.8)
                                                                      .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                                .withControllerHeadingAxis(() ->
                                                                                                                Math.sin(
                                                                                                                    driverXbox.getRawAxis(
                                                                                                                        2) *
                                                                                                                    Math.PI) *
                                                                                                                (Math.PI *
                                                                                                                2),
                                                                                                            () ->
                                                                                                                Math.cos(
                                                                                                                    driverXbox.getRawAxis(
                                                                                                                        2) *
                                                                                                                    Math.PI) *
                                                                                                                (Math.PI *
                                                                                                                2))
                                                                                .headingWhile(false);


                                                                                /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(){
      
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);

      // motor1.setDefaultCommand(Commands.run(()-> motor1.setmotor(()-> 0), motor1));
      NamedCommands.registerCommand("leftalignment",new autoalignment(drivebase,ELEVATOR,0.51,-0.16));// 75 16
      NamedCommands.registerCommand("rightalignment",new autoalignment(drivebase,ELEVATOR,0.51,0.16));
      NamedCommands.registerCommand("l4", new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l4).until(()-> ElevatorConst.getfinish() == 1));
      NamedCommands.registerCommand("underalgae", new AutoActuation(ELEVATOR, PIVOT, upsystemstates.underalgae).until(()-> ElevatorConst.getfinish() == 1));


      NamedCommands.registerCommand("backfeed", new backfeed(drivebase));
      NamedCommands.registerCommand("backfeed2", new backfeed2(drivebase));

      NamedCommands.registerCommand("source", new AutoActuation(ELEVATOR, PIVOT, upsystemstates.source).withTimeout(1.3));
      NamedCommands.registerCommand("halfsource", new AutoActuation(ELEVATOR, PIVOT, upsystemstates.source).withTimeout(0.53));

      NamedCommands.registerCommand("score",  new HardScore(CATCHER_LEFT, CATCHER_RIGHT).withTimeout(0.7));
      NamedCommands.registerCommand("score2",  new HardScore(CATCHER_LEFT, CATCHER_RIGHT).withTimeout(0.7));

      // NamedCommands.registerCommand("intake",  new intake(CATCHER_LEFT, CATCHER_RIGHT).withTimeout(1.25));
      NamedCommands.registerCommand("intake",  new SequentialCommandGroup(new intake(CATCHER_LEFT, CATCHER_RIGHT).until(()-> CATCHER_RIGHT.hascoral()),
      new intake(CATCHER_LEFT, CATCHER_RIGHT).withTimeout(0.2)));

      NamedCommands.registerCommand("catchalgae",  new SequentialCommandGroup(new intake(CATCHER_LEFT, CATCHER_RIGHT).withTimeout(4)));

      NamedCommands.registerCommand("centeralignment",new autoalignment(drivebase,ELEVATOR,0.65,0));

      NamedCommands.registerCommand("1secondintake",  new intake(CATCHER_LEFT, CATCHER_RIGHT).withTimeout(.61));

    }
  
    private void configureBindings()
    {
      CATCHER_RIGHT.setDefaultCommand(Commands.run(()-> CATCHER_RIGHT.setmotor(0), CATCHER_RIGHT));
      CATCHER_LEFT.setDefaultCommand(Commands.run(()-> CATCHER_LEFT.setmotor(0), CATCHER_LEFT));

      PIVOT.setDefaultCommand(Commands.run(()-> PIVOT.setState(Wriststates.MANUAL), PIVOT));

      ELEVATOR.setDefaultCommand(Commands.run(()-> ELEVATOR.setState(ElevatorStates.MANUAL), ELEVATOR));

      CLIMB.setDefaultCommand(Commands.run(()-> CLIMB.setmotor(()-> 0),CLIMB));
          

      @SuppressWarnings("unused")
      Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      @SuppressWarnings("unused")
      Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
      @SuppressWarnings("unused")
      Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
      Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
      Command driveRobotOriented = Commands.run(()-> drivebase.drive(new ChassisSpeeds(-1.1*driverXbox.getLeftY(), -0.8*driverXbox.getLeftX(),-1.1*driverXbox.getRightX())));
      Command fastdriveR = drivebase.driveFieldOriented(fastdrive);
      

      if (RobotBase.isSimulation()){
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
      }else{
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      }


      if (Robot.isSimulation()){
        driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      }if (DriverStation.isTest()){
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); 

        driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.back().whileTrue(drivebase.centerModulesCommand());
        driverXbox.leftBumper().onTrue(Commands.none());
        driverXbox.rightBumper().onTrue(Commands.none());   
      }else{
        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.back().whileTrue(new AimatCoralStation(drivebase));

        driverXbox.leftTrigger(.04).whileTrue(Commands.runOnce(drivebase::lock, drivebase).andThen( driveRobotOriented)); 
        driverXbox.rightTrigger(.04).whileTrue(fastdriveR);

        driverXbox.rightBumper().whileTrue(Commands.run(()-> motor1.setmotor(()-> -0.5), motor1));
        driverXbox.leftBumper().whileTrue(Commands.run(()-> motor1.setmotor(()-> 0.5), motor1));

        driverXbox.rightBumper().whileTrue(new autoalignment(drivebase,ELEVATOR, 0.61, 0.16));
        driverXbox.leftBumper().whileTrue(new autoalignment(drivebase,ELEVATOR,0.61,-0.16));// 0.61 18
        driverXbox.b().whileTrue(new centeralignment(drivebase,ELEVATOR,0.51,0));
        driverXbox.x().whileTrue(new centeralignmentx(drivebase,ELEVATOR,0.51,0));


        driverXbox.y().whileTrue(Commands.run(()-> drivebase.driveToPose(new Pose2d(7.8, drivebase.getPose().getY(),Rotation2d.fromDegrees(180))), drivebase));
        driverXbox.button(10).whileTrue(new NetAlignment(drivebase,0.4));

        driverXbox.povDown().whileTrue(Commands.run(()-> CLIMB.setmotor(()-> -0.5), CLIMB));
        // driverXbox.povUp().whileTrue(Commands.run(()-> CLIMB.setmotor(()-> 0.4), CLIMB));

        driverXbox2.x().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l3));
        driverXbox2.a().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l4));
        driverXbox2.b().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l2));
        driverXbox2.y().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.source));

        driverXbox2.povDown().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.underalgae));
        driverXbox2.povUp().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.upperalgae));
        driverXbox2.povRight().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.lb2));
        driverXbox2.povLeft().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.net));

        driverXbox2.rightTrigger(0.06).whileTrue(Commands.run(()-> CATCHER_RIGHT.setrightmotor(()->driverXbox2.getRightTriggerAxis()), CATCHER_RIGHT));
        driverXbox2.leftTrigger(0.06).whileTrue(Commands.run(()-> CATCHER_LEFT.setleftmotor(()->-driverXbox2.getLeftTriggerAxis()), CATCHER_LEFT));

        driverXbox2.rightBumper().whileTrue( new Score(CATCHER_LEFT, CATCHER_RIGHT));
        driverXbox2.back().whileTrue(new DelayedScore(CATCHER_LEFT, CATCHER_RIGHT));
        driverXbox2.leftBumper().whileTrue( new HardScore(CATCHER_LEFT, CATCHER_RIGHT));

    }}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
      // return null;
      // return Commands.run(()-> drivebase.driveFieldOriented(new ChassisSpeeds(1, 0, 0.1))).withTimeout(1);




       return drivebase.getAutonomousCommand("left4");
      // return null;


      //  return drivebase.getAutonomousCommand("center3");
    }

    public void setMotorBrake(boolean brake){
      drivebase.setMotorBrake(brake);
    }}  