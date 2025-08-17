package frc.robot.commands.swervedrive.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;;;
/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * <a href="https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java">...</a>
 */
public class FinalAlignment extends Command
{

  private final SwerveSubsystem swerveSubsystem;
  private final Pose2d targetPose;
  private Supplier<Pose2d> currentPose; 
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  
  public FinalAlignment(SwerveSubsystem swerveSubsystem, Pose2d targetPose, Supplier<Pose2d> currentPose)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.targetPose = targetPose;
    this.currentPose = currentPose;
    
    xController = new PIDController(1.5, 0.2, 0.0);
    yController = new PIDController(1.5, 0.2, 0.0);
    rotationController = new PIDController(1.0, 0.1, 0.0);
    
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    rotationController.setTolerance(2.0);
    
    rotationController.enableContinuousInput(-180, 180);
    
    addRequirements(swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    Pose2d current = currentPose.get();
    
    double xError = targetPose.getX() - current.getX();
    double yError = targetPose.getY() - current.getY();
    
    double rotError = targetPose.getRotation().minus(current.getRotation()).getDegrees();
    if (rotError > 180) rotError -= 360;
    if (rotError < -180) rotError += 360;
    
    double xSpeed = xController.calculate(-xError, 0);
    double ySpeed = yController.calculate(-yError, 0);
    double rotSpeed = rotationController.calculate(rotError, 0);
    
    double minSpeed = 0.015;
    double errorThreshold = 0.02;
    
    if (Math.abs(xError) > errorThreshold) {
        xSpeed = Math.copySign(Math.max(Math.abs(xSpeed), minSpeed), xSpeed);
    } else {
        xSpeed = 0;
    }
    
    if (Math.abs(yError) > errorThreshold) {
        ySpeed = Math.copySign(Math.max(Math.abs(ySpeed), minSpeed), ySpeed);
    } else {
        ySpeed = 0;
    }
    
    if (Math.abs(rotError) > rotationController.getErrorTolerance()) {
        rotSpeed = Math.copySign(Math.max(Math.abs(rotSpeed), minSpeed), rotSpeed);
    } else {
        rotSpeed = 0;
    }
    
    xSpeed = MathUtil.clamp(xSpeed, -0.3, 0.3);
    ySpeed = MathUtil.clamp(ySpeed, -0.3, 0.3);
    rotSpeed = MathUtil.clamp(rotSpeed, -0.3, 0.3);
    
    swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, true);
    
    SmartDashboard.putNumber("Alignment X Error", xError);
    SmartDashboard.putNumber("Alignment Y Error", yError);
    SmartDashboard.putNumber("Alignment Rot Error", rotError);
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Rot Speed", rotSpeed);

    SmartDashboard.putBoolean(" x at setpoint", xController.atSetpoint());  
    SmartDashboard.putBoolean(" y at setpoint", yController.atSetpoint());  
    SmartDashboard.putBoolean(" z at setpoint", rotationController.atSetpoint());  
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return xController.atSetpoint() && 
           yController.atSetpoint() && 
           rotationController.atSetpoint();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    swerveSubsystem.lock();
  }
}