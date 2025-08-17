package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConst;;

public class Elevator extends SubsystemBase {

  private final Encoder encoder = new Encoder(0, 1);
  private double setpoint;

  private double error= 0;

  // private double startTime = 0;
  // private double previousError = 0.0;  // Önceki hatayı saklamak için

  // private boolean firstMove = true;

    public static enum ElevatorStates {
      NONE,
      zero,
      L2,
      L3,
      L4,
      underalgae,
      upperalgae,
      net,
      MANUAL
        }

    private ElevatorStates state = ElevatorStates.NONE;

  /**
   * Elevator States: Different states for the elvator to be in to do. They do different things in each state to run to a specific position for a function.
   * Use setState to set to an elevator state, and make sure to import elevatorStates in the command so you can use the state to put it in.
*/

  private int elevatorleftMotorID = 10;
  private int elevatorrightMotorID = 12;

  private TalonFX elevatorMotor = new TalonFX(elevatorleftMotorID);
  private TalonFX elevatorFollowerMotor = new TalonFX(elevatorrightMotorID);
  
  private DoubleSupplier motorout;

  public Elevator(DoubleSupplier motorout) { 
    this.motorout = motorout;
    configMotors();
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator distance", getheight());
    SmartDashboard.putNumber("finish", ElevatorConst.finishvalue);
    SmartDashboard.putNumber("get el error", Math.abs(geterror()));

    if (Math.abs(geterror()) < 0.1 && Math.abs(geterror()) != 0) {
      ElevatorConst.changefinish(1);
     }else{
      ElevatorConst.changefinish(0);
     }
    // if(Wrist.elevatorcanfly()){
      switch (state) {
        case zero:
          pid( Constants.ElevatorConst.zeroHeight);
         break;
         case L2:
         setpoint = Constants.ElevatorConst.L2Height;
           pid(setpoint);
          break;
        case L3:
        setpoint = Constants.ElevatorConst.L3Height;
          pid(setpoint);
         break;
        case L4:
        setpoint = Constants.ElevatorConst.L4Height;
        pid(setpoint);
        break;
        case net:
        setpoint = Constants.ElevatorConst.netheight;
        pid(setpoint);
        break;
        case MANUAL:  
        manual(this.motorout,0.05);
         break;
        case underalgae:
        setpoint = Constants.ElevatorConst.underalgaeHeight;
        pid(setpoint);
        break;
        case upperalgae:
        setpoint = Constants.ElevatorConst.upperalgaeHeight;
        pid(setpoint);
        break;
        default:
        break;
      // }}else{
      //   switch (state) {
      //   case MANUAL:
      //   manual(this.motorout);
      //  break;
      // default:
      //  break;
      }
    }
       

    public void manual(DoubleSupplier motorout,double deadband) {
      double motorOutput = MathUtil.applyDeadband(motorout.getAsDouble(), 0.05);
  
      if(motorOutput < 0){
       motorOutput = motorOutput * 0.4;
      }else{
       motorOutput = motorOutput * 0.15;
      }

      elevatorMotor.set(motorOutput);
      elevatorFollowerMotor.set(-motorOutput);
      SmartDashboard.putNumber("manualmotorspeed", motorOutput);
    }

  public void manual(DoubleSupplier motorout) {
  
    double motorOutput = motorout.getAsDouble() ;

    if(motorOutput < 0){
    motorOutput = motorOutput * 0.4;
    }else{
      motorOutput = motorOutput * 0.15;
    }

    elevatorMotor.set(motorOutput);
    elevatorFollowerMotor.set(-motorOutput);
    SmartDashboard.putNumber("manualmotorspeed", motorOutput);
    
  }

  public double getheight(){
    return encoder.getDistance()*0.000083;
  }

  public void pid(double setpoint) {
    try (PIDController controller = new PIDController( 
        Constants.ElevatorConst.kPIDElevator.kP, 
        0,  
        Constants.ElevatorConst.kPIDElevator.kD)) 
    {
        controller.setSetpoint(setpoint);
         error = setpoint - getheight();
        
        double elevatorUpSpeed = .36;
        double elevatorDownSpeed = 0.18;
        double brakeSpeed = 0.037;

        double calculatedMotorSpeed;
        int currentPhase = 0;

        // double ERROR_THRESHOLD = 0.05;

        // **Yön değişimi kontrolü (Tolerans ile)**
        // if ((previousError < -ERROR_THRESHOLD && error > ERROR_THRESHOLD) || 
        //     (previousError > ERROR_THRESHOLD && error < -ERROR_THRESHOLD)) {
        //     firstMove = true;  // Eğer yön değiştiyse, hareketi sıfırla
        // }

        // if (firstMove) {
        //     startTime = Timer.getFPGATimestamp();  
        //     firstMove = false;  
        // }

        if (error < 0.0) {  // Aşağı hareket
            if (Math.abs(error) > 0.1) {
                currentPhase = 1;
                // calculatedMotorSpeed = isInitialWait(startTime) ? -brakeSpeed : elevatorDownSpeed;  
                calculatedMotorSpeed = elevatorDownSpeed;
            } else {
                currentPhase = 2;
                calculatedMotorSpeed = -brakeSpeed; 
            }
        } else {  // Yukarı hareket
            if (error < 0.1) {
                currentPhase = 3;
                calculatedMotorSpeed = -brakeSpeed;
            } else if (error > 0.1 && error < 0.23) {
                currentPhase = 4;
                // calculatedMotorSpeed = isInitialWait(startTime) ? -brakeSpeed : -elevatorUpSpeed * error * 25;
                calculatedMotorSpeed =  -elevatorUpSpeed * error * 3.3  ;
            } else {
                currentPhase = 5;
                // calculatedMotorSpeed = isInitialWait(startTime) ? -brakeSpeed : -elevatorUpSpeed;
                calculatedMotorSpeed =  -elevatorUpSpeed;
            }
        }

        elevatorMotor.set(calculatedMotorSpeed);
        elevatorFollowerMotor.set(-calculatedMotorSpeed);

        SmartDashboard.putNumber("Current Phase", currentPhase);
        SmartDashboard.putNumber("Calculated Motor Speed", calculatedMotorSpeed);
        SmartDashboard.putNumber("Elevator Error", error);

        // previousError = error; 
    }
}

// private boolean isInitialWait(double startTime) {
//     return Timer.getFPGATimestamp() - startTime < 0.4 && getheight() > 0.13;
// }


  private void configMotors() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    elevatorMotor.getConfigurator().apply(motorConfig);
    elevatorFollowerMotor.getConfigurator().apply(motorConfig);
  }

  public double geterror(){
    return setpoint - getheight() ;
  }
  public double getzeroerror(){
    return 0.03 - getheight() ;
  }
  public double getuppesterror(){
    return getheight() - 1.124 ;
  }
  
    /**
   * Sets the state of the elevator
   * @param state
   */
  public void setState(ElevatorStates state) {
    this.state = state;
  }

}