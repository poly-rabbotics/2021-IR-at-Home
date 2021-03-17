package frc.robot.subsystems;
import frc.robot.RobotMap;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousDrive {
    public static CANEncoder leftFront, rightFront, leftBack, rightBack;
    public static double inchesToTravel, encoderCountsPerInch, encoderCountsPer360;

    public AutonomousDrive() {
        leftFront = RobotMap.leftFrontEncoder;
        rightFront = RobotMap.rightFrontEncoder;
        leftBack = RobotMap.leftBackEncoder;
        rightBack = RobotMap.rightBackEncoder;
        encoderCountsPerInch = 0.27; 
        encoderCountsPer360 = 20.8;
        
    }
    public void printState() {
        SmartDashboard.putNumber("leftFront Encoder Counts", leftFront.getPosition());
        SmartDashboard.putNumber("rightFront Encoder Counts", rightFront.getPosition());
        SmartDashboard.putNumber("leftBack Encoder Counts", leftBack.getPosition());
        SmartDashboard.putNumber("rightBack Encoder Counts", rightBack.getPosition());
        
      }

    public static void DriveByDistance(double inchesToTravel) {
        leftFront.setPosition(leftFront.getPosition()+(inchesToTravel * encoderCountsPerInch));
        rightFront.setPosition(-(rightFront.getPosition()+(inchesToTravel * encoderCountsPerInch)));
        leftBack.setPosition(leftBack.getPosition()+(inchesToTravel * encoderCountsPerInch));
        rightBack.setPosition(-(rightBack.getPosition()+(inchesToTravel * encoderCountsPerInch)));
    }
    
    public static void Turn(double degrees) {
        leftFront.setPosition(leftFront.getPosition()+(degrees * (encoderCountsPer360/360) ) );
        rightFront.setPosition(-(rightFront.getPosition()+(degrees * (encoderCountsPer360/360) ) ) );
        leftBack.setPosition(leftBack.getPosition()+(degrees * (encoderCountsPer360/360) ) );
        rightBack.setPosition(-(rightBack.getPosition()+(degrees * (encoderCountsPer360/360) ) ) );
    }

}
