package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.util.Conversions;
import frc.util.PIDConst;

public class Constants {
   
    public static class ElevatorConstants
    {
  
      public static final double kElevatorKp = 5;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0;
  
      public static final double kElevatorkS = 0.0; // volts (V)
      public static final double kElevatorkG = 0.762; // volts (V)
      public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
      public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  
      public static final double kElevatorGearing = 6.0;
      public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
      public static final double kCarriageMass = 4.0; // kg
  
  
      public static final double kRotaionToMeters = kElevatorDrumRadius * 2 * Math.PI;
      public static final double kRPMtoMPS = (kElevatorDrumRadius * 2 * Math.PI) / 60;
      public static final double kMinElevatorHeightMeters = 0.0;
      public static final double kMaxElevatorHeightMeters = 1.34705;
  
      public static final double kElevatorMaxVelocity = 3.5;
      public static final double kElevatorMaxAcceleration = 2.5;
    }
  
    
    public static final class ElevatorConst {
      
      public static double zeroHeight = 0.02; // 0.14 // 0.22; // 0.21; // 0.13
      public static double L2Height = 0.24; // 0.11;
      public static double L3Height = 0.61  ; // 0.58;
      public static double L4Height = 1.119; // 0.11;
      public static double underalgaeHeight = 0.32; // 0.11;
      public static double upperalgaeHeight = 0.67; // 0.11;
      public static double netheight = 1.135; // 0.11;
  
  
  
      public static final PIDConst kPIDElevator = new PIDConst(2.4 , 0.0, 0.03, 0.0);
      public static final double kElevatorVelocity = 0.35; // 0.25
      public static final double kElevatorMotorMaxOutput = 0.4;
      public static double kElevatorMotorMaxOutputClimb = 0.45; // 0.2;
  
      
      public static final double kAtGoalTolerance = 0.01;
  
      public static final double kElevatorWinchDiameterM = Conversions.inchesToMeters(1.5);
      public static final double kElevatorWinchCircumM = kElevatorWinchDiameterM * Math.PI;
  
      public static final boolean kLeftMotorIsInverted = true; // true
      public static final boolean kRightMotorIsInverted = false; // false
  
      public static int finishvalue = 0; 
  
      public static void changefinish(int newFinishValue) {
          finishvalue = newFinishValue; 
      }
  
      public static int getfinish() {
          return finishvalue; 
      }
  } 
  
  public static final class Pivot {
      public static double zeroangle = 46;//46
      // public static double l4angle = 48;
  
      public static double algeaangle = 70; 
      public static double sourceangle = 136; 
      public static double l2b = 90; 
      public static double stucksource = 128; 
      public static double netangle = 137; 
  
  
  
      
      public static final PIDConst kfastPIDPivot = new PIDConst(0.015, 0.0, 0.0);
      public static final PIDConst kslowPIDPivot = new PIDConst(0.0034, 0.0, 0.0);
  }
}
