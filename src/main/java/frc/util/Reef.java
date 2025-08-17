package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Reef {

    public static class coralStationConstants {
    public static final double fieldxdistance = 17.548;
    public static final double fieldydistance = 8.05;
    public static final double fieldRotation = 180;

    public static final double ABstationX = 3.658;
    public static final double CDstationX = 4.074;
    public static final double EFstationX = 4.905;
    public static final double GHstationX = 5.321;
    public static final double IJstationX = 4.905;
    public static final double KLstationX = 4.074;
    
    public static final double ABstationY = 4.026;
    public static final double CDstationY = 3.306;
    public static final double EFstationY = 3.306;
    public static final double GHstationY = 4.026;
    public static final double IJstationY = 4.745;
    public static final double KLstationY = 4.745;

    public static final double ABstationAngle=180;
    public static final double CDstationAngle=-120;
    public static final double EFstationAngle=-60;
    public static final double GHstationAngle=0;
    public static final double IJstationAngle=60;
    public static final double KLstationAngle=120;

    public static final double ABstationRobotAngle = 0;     
    public static final double CDstationRobotAngle = 60;    
    public static final double EFstationRobotAngle = 120;   
    public static final double GHstationRobotAngle = 180;   
    public static final double IJstationRobotAngle = 240;
    public static final double KLstationRobotAngle = 300;    

    public static final Pose2d ABstationPose = new Pose2d(ABstationX, ABstationY, new Rotation2d(Math.toRadians(ABstationAngle)));
    public static final Pose2d CDstationPose = new Pose2d(CDstationX, CDstationY, new Rotation2d(Math.toRadians(CDstationAngle)));
    public static final Pose2d EFstationPose = new Pose2d(EFstationX, EFstationY, new Rotation2d(Math.toRadians(EFstationAngle)));
    public static final Pose2d GHstationPose = new Pose2d(GHstationX, GHstationY, new Rotation2d(Math.toRadians(GHstationAngle)));
    public static final Pose2d IJstationPose = new Pose2d(IJstationX, IJstationY, new Rotation2d(Math.toRadians(IJstationAngle)));
    public static final Pose2d KLstationPose = new Pose2d(KLstationX, KLstationY, new Rotation2d(Math.toRadians(KLstationAngle)));

      public enum Station {
        AB, 
        CD, 
        EF, 
        GH, 
        IJ, 
        KL;
        public static Station fromString(String name) {
          for (Station station : values()) {
              if (station.name().equalsIgnoreCase(name)) {
                  return station;
              }
          }
          return null;
        }
        public int getStationTagId() {
            int AprilTagID;
            switch (this) {
                case AB:
                    AprilTagID = 18;
                    break;
                case CD:
                    AprilTagID = 17;
                    break;
                case EF:
                    AprilTagID = 22;
                    break;
                case GH:
                    AprilTagID = 21;
                    break;
                case IJ:
                    AprilTagID = 20;
                    break;
                case KL:
                    AprilTagID = 19;
                    break;
                default:
                    AprilTagID = -1; 
                    break;
            }
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {

                int mod = (int) AprilTagID % 3;
                if (mod == 0) {
                    AprilTagID -= 11;
                } else if (mod == 1) {
                    AprilTagID -= 13;
                } else if (mod == 2) {
                    AprilTagID -= 9;
                }}}
            
            return AprilTagID;
            }
        

            public int getStationTagIdx() {
                int AprilTagID;
                switch (this) {
                    case AB:
                        AprilTagID = 18;
                        break;
                    case CD:
                        AprilTagID = 17;
                        break;
                    case EF:
                        AprilTagID = 22;
                        break;
                    case GH:
                        AprilTagID = 21;
                        break;
                    case IJ:
                        AprilTagID = 20;
                        break;
                    case KL:
                        AprilTagID = 19;
                        break;
                    default:
                        AprilTagID = -1; // Varsayılan bir değer, eğer hiçbir koşul sağlanmazsa
                        break;
                }
                if (DriverStation.getAlliance().isPresent()) {
                    if (DriverStation.getAlliance().get() == Alliance.Blue) {
    
                    int mod = (int) AprilTagID % 3;
                    if (mod == 0) {
                        AprilTagID -= 11;
                    } else if (mod == 1) {
                        AprilTagID -= 13;
                    } else if (mod == 2) {
                        AprilTagID -= 9;
                    }}}
                
                return AprilTagID;
                }
            
/**
 * Get the X position of the selected station.
 *
 * @param station The station to get the X position for.
 * @return The X position of the station, flipped if the alliance is Red.
 */
public double getStationX() {
  double flipper = coralStationConstants.fieldxdistance;
  double stationX;

  // İstasyonun X konumunu belirleyin
  switch (this) {
      case AB:
          stationX = coralStationConstants.ABstationX;
          break;
      case CD:
          stationX = coralStationConstants.CDstationX;
          break;
      case EF:
          stationX = coralStationConstants.EFstationX;
          break;
      case GH:
          stationX = coralStationConstants.GHstationX;
          break;
      case IJ:
          stationX = coralStationConstants.IJstationX;
          break;
      case KL:
          stationX = coralStationConstants.KLstationX;
          break;
      default:
          stationX =0;
  }

  if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
          return stationX;  // blue alliance
      } else {
          double flippedvalue = Math.abs(flipper - stationX);
          return flippedvalue;// red alliance
      }
  } else {
      return 0;
  }
}

public double getStationXx() {
    double flipper = coralStationConstants.fieldxdistance;
    double stationX;
  
    // İstasyonun X konumunu belirleyin
    switch (this) {
        case AB:
            stationX = coralStationConstants.ABstationX;
            break;
        case CD:
            stationX = coralStationConstants.CDstationX;
            break;
        case EF:
            stationX = coralStationConstants.EFstationX;
            break;
        case GH:
            stationX = coralStationConstants.GHstationX;
            break;
        case IJ:
            stationX = coralStationConstants.IJstationX;
            break;
        case KL:
            stationX = coralStationConstants.KLstationX;
            break;
        default:
            stationX =0;
    }
  
    if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return stationX;  // blue alliance
        } else {
            double flippedvalue = Math.abs(flipper - stationX);
            return flippedvalue;// red alliance
        }
    } else {
        return 0;
    }
  }

 /**
 * Get the Y position of the selected station based on the alliance.
 *
 * @param station The station to get the Y position for.
 * @return The Y position of the station, flipped if the alliance is Red.
 */
public double getStationY() {
  double flipper = coralStationConstants.fieldydistance;
  double stationY;

  switch (this) {
      case AB:
          stationY = coralStationConstants.ABstationY;
          break;
      case CD:
          stationY = coralStationConstants.CDstationY;
          break;
      case EF:
          stationY = coralStationConstants.EFstationY;
          break;
      case GH:
          stationY = coralStationConstants.GHstationY;
          break;
      case IJ:
          stationY = coralStationConstants.IJstationY;
          break;
      case KL:
          stationY = coralStationConstants.KLstationY;
          break;
      default:
          throw new IllegalArgumentException("Unknown station: " + this);
  }

  if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
          return stationY;  // Blue alliance
      } else {
          double flippedvalue = Math.abs(flipper - stationY);
          return flippedvalue;  // Red alliance
      }
  } else {
      return 0;  
  }
}
public double getStationYy() {
    double flipper = coralStationConstants.fieldydistance;
    double stationY;
  
    switch (this) {
        case AB:
            stationY = coralStationConstants.ABstationY;
            break;
        case CD:
            stationY = coralStationConstants.CDstationY;
            break;
        case EF:
            stationY = coralStationConstants.EFstationY;
            break;
        case GH:
            stationY = coralStationConstants.GHstationY;
            break;
        case IJ:
            stationY = coralStationConstants.IJstationY;
            break;
        case KL:
            stationY = coralStationConstants.KLstationY;
            break;
        default:
            throw new IllegalArgumentException("Unknown station: " + this);
    }
  
    if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return stationY;  // Blue alliance
        } else {
            double flippedvalue = Math.abs(flipper - stationY);
            return flippedvalue;  // Red alliance
        }
    } else {
        return 0;  
    }
  }
        public double getRotationCoralStation() {
          double rotation;
    
          switch (this) {
            case AB:
                rotation = coralStationConstants.ABstationRobotAngle;
                break;
            case CD:
                rotation = coralStationConstants.CDstationRobotAngle;
                break;
            case EF:
                rotation = coralStationConstants.EFstationRobotAngle;
                break;
            case GH:
                rotation = coralStationConstants.GHstationRobotAngle;
                break;
            case IJ:
                rotation = coralStationConstants.IJstationRobotAngle;
                break;
            case KL:
                rotation = coralStationConstants.KLstationRobotAngle;
                break;
            default:
                rotation = -1;
                break;
        }
        
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
         rotation = Math.abs(rotation+  coralStationConstants.fieldRotation);
         return rotation;
      }else{
        return rotation;
      }
    }else{
        return rotation;
    }
  }

  public double getRotationCoralStationx() {
    double rotation;

    switch (this) {
      case AB:
          rotation = coralStationConstants.ABstationRobotAngle;
          break;
      case CD:
          rotation = coralStationConstants.CDstationRobotAngle;
          break;
      case EF:
          rotation = coralStationConstants.EFstationRobotAngle;
          break;
      case GH:
          rotation = coralStationConstants.GHstationRobotAngle;
          break;
      case IJ:
          rotation = coralStationConstants.IJstationRobotAngle;
          break;
      case KL:
          rotation = coralStationConstants.KLstationRobotAngle;
          break;
      default:
          rotation = -1; 
          break;
  }
  
if(DriverStation.getAlliance().isPresent()){
if(DriverStation.getAlliance().get() == Alliance.Blue){
   rotation = Math.abs(rotation+  coralStationConstants.fieldRotation);
   return rotation;
}else{
  return rotation;
}
}else{
  return rotation;
    }
}}}}
