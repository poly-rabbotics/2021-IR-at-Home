package frc.robot.subsystems;

import frc.robot.RobotMap;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousDrive {
    public static CANEncoder leftFront, rightFront, leftBack, rightBack;
    public static double inchesToTravel, encoderCountsPerInch, encoderCountsPer360;
    private SpeedControllerGroup left, right;
    private static DifferentialDrive drive;
    public static boolean driveDone;

    public AutonomousDrive() {
        leftFront = RobotMap.leftFrontEncoder;
        rightFront = RobotMap.rightFrontEncoder;
        leftBack = RobotMap.leftBackEncoder;
        rightBack = RobotMap.rightBackEncoder;
        left = new SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
        right = new SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
        drive = new DifferentialDrive(right, left);
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
        double setpoint = inchesToTravel * encoderCountsPerInch + leftFront.getPosition();
        // only one encoder needs to be watched for driving forward
        while (leftFront.getPosition() < setpoint) {
            drive.tankDrive(.3, .3);
            driveDone = false;
        }
        drive.tankDrive(0, 0);
        driveDone = true;  
    }
    
    public static void Turn(double degrees) {
        double setpointLeft = degrees * (encoderCountsPer360/360) + leftFront.getPosition();
        while (leftFront.getPosition()<setpointLeft){
            if (degrees<0) {
                drive.tankDrive(-0.3,0.3);
            }
            else {
                drive.tankDrive(0.3,-0.3);
            }
            driveDone = false;
        }
        drive.tankDrive(0,0);
        driveDone = true;
    }
}
