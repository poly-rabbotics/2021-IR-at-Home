package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;

public class MechanismsJoystick {
    private static Joystick joystick = RobotMap.mechanismsJoystick;
    public static boolean isManual = false;
public static String controlState = "Normal";


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Auto Modes-----------------------------------------------------------------------------------------------------------------------------------
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean startGalacticSearch(){
    return  joystick.getRawButton(4); //needs to be given an actual number
      }
    public static boolean autoNavOne(){
      return joystick.getRawButton(3); //needs port number
    }
    public static boolean autoNavTwo(){
      return joystick.getRawButton(5); //port #
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // SHOOTER ADJUST --------------------------------------------------------------------------------------------------------------------------------
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static double getChangeTopShooter() {
      return joystick.getRawAxis(3);
    }
    public static double getChangeBottomShooter() {
      return joystick.getRawAxis(5);
    }
    public static boolean allowCalibrateShooter(){
      return joystick.getRawButton(3);
    }
  
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     // SHOOTER CONTROLS  ---------------------------------------------------------------------------------------------------------------------------
     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     public static boolean getToggleManShootOne(){
      return joystick.getRawButton(5);
    }
    public static boolean getToggleManShootTwo(){
      return joystick.getRawButton(6);
    }
  
    public static boolean getToggleManShootThree(){
      return joystick.getRawButton(7);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MANUAL ONLY COMMANDS ----------------------------------------------------------------------------------------------------------------------
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean isManual(){
      if(isManual == true){
        controlState = "Manual";
      }
      else{
        controlState = "Normal";
      }
      return joystick.getRawButton(11);
    }}

//somehow works with no bracket here?