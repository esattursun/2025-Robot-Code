package frc.util;

import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist.Wriststates;

public class Actuation {
    
    public static final class ActuationConstans {

    public static enum upsystemstates {
        NONE,
        underalgae,
        upperalgae,
        catchedalgae,
        source,
        l1,
        l2,
        l3,
        l4,
        lb2,
        net,
        MANUAL;
        
        public ElevatorStates getelevatorstate(){        
            switch (this) {
                case underalgae:
                    return ElevatorStates.underalgae;
                case upperalgae:
                    return ElevatorStates.upperalgae;
                case source,lb2:
                    return ElevatorStates.zero;
                case l1:
                case l2:
                    return ElevatorStates.L2;
                case l3:
                    return ElevatorStates.L3;
                case l4:
                    return ElevatorStates.L4;
                case MANUAL:
                    return ElevatorStates.MANUAL;
                default:
                    return ElevatorStates.NONE;
            }
    
      }

      public Wriststates getwriststate(){
        switch (this) {
            case underalgae, upperalgae, l1, l2, l3,l4:
                return Wriststates.zero;
            case source:
                return Wriststates.source;
            case lb2:
                return Wriststates.lb2;
            case MANUAL:
                return Wriststates.MANUAL;
            default:
                return Wriststates.NONE;
        }
  }

}
}
}
