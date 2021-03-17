package frc.robot.Controls;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;

public class MechanismsJoystick {
    private static Joystick joystick = RobotMap.mechanismsJoystick;


    public static boolean startGalacticSearch(){
    return  joystick.getRawButton(4); //needs to be given an actual number
      }
    public static boolean autoNavOne(){
      return joystick.getRawButton(3); //needs port number
    }
    public static boolean autoNavTwo(){
      return joystick.getRawButton(5); //port #
    }

}