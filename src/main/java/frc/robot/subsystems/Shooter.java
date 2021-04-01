package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter {

    private TalonSRX topMotor, bottomMotor;
    private double topMotorSpeed, bottomMotorSpeed, LIDARNumberTop, LIDARNumberBottom;
    private Limelight Limelight;
    private SpeedControllerGroup left, right;
    private DifferentialDrive drive;
    public Shooter() {
        Limelight = new Limelight();
        topMotorSpeed = 0;
        bottomMotorSpeed = 0;
        topMotor = RobotMap.shooterTopMotor;
        bottomMotor = RobotMap.shooterBottomMotor; 
        LIDARNumberTop = 10; //DETERMINE FROM CALCULATIONS AND TESTING
        LIDARNumberBottom = 10; //DETERMINE FROM CALCULATIONS AND TESTING
        //drive objects are necessary for automated shooting sequence
        left = new SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
        right = new SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
        drive = new DifferentialDrive(right, left);
        //distance = 0.0;
        //preset = 1;
        //lowSpeed = 0.225;
        //highSpeed = 0.525;
        //bothSpeed = 0.7;
        //solenoid = RobotMap.shooterSolenoid;
        //solenoidOut = false;
        //configureShooterPids();
    }
    public void adjustSpeeds(){
        if(MechanismsJoystick.getChangeTopShooter() > 0.1 && topMotorSpeed <= 1) {
          topMotorSpeed += .005;
        }
        else if(MechanismsJoystick.getChangeTopShooter() < -0.1 && topMotorSpeed >= 0) {
          topMotorSpeed -= .005;
        }
        if(MechanismsJoystick.getChangeBottomShooter() > .1 && bottomMotorSpeed <= 1) {
          bottomMotorSpeed -= .005;
        }
        else if(MechanismsJoystick.getChangeBottomShooter() <- .1 && bottomMotorSpeed >= 0) {
          bottomMotorSpeed -= .005;
        }
    }

    
    double topMotorV = topMotorSpeed * 2048 / 600;
    double bottomMotorV = bottomMotorSpeed * 2048 / 600;

    public void calibrateShooter(){
  
      if (MechanismsJoystick.getToggleManShootOne()){
        bottomMotorSpeed += 10;
      }
      if (MechanismsJoystick.getToggleManShootTwo()){
        topMotorSpeed -= 10;
      }
  
      if (MechanismsJoystick.getToggleManShootThree()){
        topMotorSpeed += 10;
        bottomMotorSpeed -= 10;
      }
    }
    public void autoShootSequence(){
      double tx = Limelight.getX();
      boolean aimLocked = false;
      if (tx>0){
        //if robot heading is too far right, turn left until it is good
        while (tx > 0){
          drive.tankDrive(-0.3, 0.3);
        }
        aimLocked = true;
        drive.tankDrive(0, 0);
      } else if (tx<0) {
        //if robot heading is too far left, turn right until it is good
        while (tx < 0){
          drive.tankDrive(0.3, -0.3);
        }
        aimLocked = true;
        drive.tankDrive(0, 0);
      } else {
        //if robot heading is already good, stop motors
        aimLocked = true;
        drive.tankDrive(0, 0);
      }
      topMotorSpeed = LIDAR.getDistance() * LIDARNumberTop; //insert lidar calculation here
      bottomMotorSpeed = LIDAR.getDistance() * LIDARNumberBottom;//insert lidar calculation here
      //conversion from percent output to RPM
      double topMotorV = topMotorSpeed * 2048 / 600;
      double bottomMotorV = bottomMotorSpeed * 2048 / 600;
      //engage shooter motors once everything is adjusted correctly
      topMotor.set(ControlMode.Velocity,topMotorV);
      bottomMotor.set(ControlMode.Velocity,bottomMotorV);
    }
    

    public void run() {
 
        if(MechanismsJoystick.allowCalibrateShooter()){
            calibrateShooter();
          }
          else{
            
            if(MechanismsJoystick.isManual()) {
                adjustSpeeds();
                SmartDashboard.putNumber("Top Shooter:", topMotorSpeed);
                SmartDashboard.putNumber("Bottom Shooter:", bottomMotorSpeed);
                topMotor.set(ControlMode.PercentOutput, topMotorSpeed);
                bottomMotor.set(ControlMode.PercentOutput, bottomMotorSpeed);
                /* NORMAL MANUAL SHOOTING, DO NOT USE YET
                if (MechanismsJoystick.getToggleManShootOne()) {
                  autoShootSequence();//
                }
                else {
                  topMotor.set(ControlMode.PercentOutput, 0);
                  bottomMotor.set(ControlMode.PercentOutput, 0);
                  solenoid.set(Value.kForward);
                }

                */
    
            }
        }
    }
}