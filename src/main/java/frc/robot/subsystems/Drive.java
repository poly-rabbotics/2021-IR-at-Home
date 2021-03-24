/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//direct copy from 2020 code atm
package frc.robot.subsystems;
  
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.CameraSetDriveSetpoint;
import frc.robot.Controls.DriveJoystick;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;

import frc.robot.utils.*;
import frc.robot.sensors.ShooterCamera;
/**
 * Add your docs here.
 */
public class Drive extends AutoSubsystem {
  private SpeedControllerGroup left, right;
  private DifferentialDrive drive;
  private AHRS ahrs;
  private KGains kGains;
  private PIDController turnController;
  static boolean shooterFront;
  private double speed, rotation, slowSpeed;
  private ShooterCamera camera;
  public static String front;
  private DigitalOutput relay;
  private CANEncoder testEncoder;
  
  public Drive() {
    left = new SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
    right = new SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
    relay = RobotMap.lightRelay;
    relay.set(false);
    shooterFront = true;
    drive = new DifferentialDrive(right, left);
    ahrs = RobotMap.ahrs;
    kGains = new KGains(0.01, 0.002, 1, 0);
    turnController = new PIDController(kGains, true);
    camera = new ShooterCamera(RobotMap.shooterCameraName);
    speed = 0.3;
    slowSpeed = 0.1;
    rotation = 0.1;
    front = "Shooter";
    

    reset();
  }
  public void reset() {
    turnController.reset();
    turnController.setSetpoint(ahrs.getAngle());
  }
  public void setRotationalSetpoint(double change) {
    turnController.setSetpoint(turnController.getSetpoint() + change);
  }
  public void setDriveForward(double speed) {
    //System.out.print("Setting drive forward to ");
    //System.out.println(speed);
    this.speed = speed;
  }
  public boolean aligned(double tolerance) {
    return Math.abs(ahrs.getAngle() - turnController.getSetpoint()) < tolerance;
  }
  private void autoOrient() {
    rotation = turnController.calculate(ahrs.getAngle());
  }
  public double getAngle() {
    return ahrs.getAngle();
  }
  public double getSetpoint() {
    return turnController.getSetpoint();
  }
  public double getError() {
    return getAngle() - getSetpoint();
  }
  private void move() {
    drive.arcadeDrive(speed, rotation);
  }
  public void printState() {
    SmartDashboard.putNumber("Turn", -1);
    SmartDashboard.putNumber("Angle", ahrs.getAngle());
    SmartDashboard.putNumber("setpoint", turnController.getSetpoint());
    SmartDashboard.putNumber("Accumulated Error", turnController.getAccumulatedError());
    SmartDashboard.putBoolean("Shooter is Front: ", shooterFront);
    SmartDashboard.putNumber("Encoder Counts", testEncoder.getPosition());
    
  }
  public void cameraOrient() {
    setRotationalSetpoint(camera.getYaw());
  }
  private void getControllerInput() {
    if(DriveJoystick.getStartAutoOrientLeft()) {
      setRotationalSetpoint(5);
    }
    if(DriveJoystick.getStartAutoOrientRight()) {
      setRotationalSetpoint(-5);
    }
    if(DriveJoystick.getCameraOrient()) {
      (new CameraSetDriveSetpoint("Orient with target")).start();
    }
    if(DriveJoystick.getContinueAutoOrient()) {
      autoOrient();
    }
    
    else {
      turnController.setSetpoint(ahrs.getAngle());
      speed = shooterFront ? DriveJoystick.getMove() : -DriveJoystick.getMove();
      rotation = DriveJoystick.getTurn();
      if(DriveJoystick.getFront()) {
        shooterFront = !shooterFront;
      }
    }
   
    if(shooterFront){
      front = "Shooter";
      
    }
    else{
      front = "Intake";
      
    }
  }

  public void run() {
    System.out.println(camera.getYaw());
    printState();
    //getControllerInput();

    // MAKE THIS MAKE SENSE

    
    if(DriveJoystick.dPad()){
       //  FIX FOUR CASES
       slowSpeed = 0.1;
       if(!shooterFront){
        slowSpeed = -slowSpeed;
      }
      if(DriveJoystick.getPreciseFront()){
        left.set(slowSpeed);
        right.set(-slowSpeed);
      }
      else if(DriveJoystick.getPreciseRight()){
        left.set(slowSpeed);
        right.set(slowSpeed);
      }
      else if(DriveJoystick.getPreciseBack()){
        left.set(-slowSpeed);
        right.set(slowSpeed);
      }
      else if (DriveJoystick.getPreciseLeft()){
        left.set(-slowSpeed);
        right.set(-slowSpeed);
      }
    }

    // MAKE THIS MAKE SENSE

    else{
    speed = DriveJoystick.getMove();
    rotation = DriveJoystick.getTurn();
    if(rotation < 0)
      rotation = -0.7*rotation*rotation;
      else
      rotation = 0.7*rotation*rotation;
    if(DriveJoystick.getFront()) {
      shooterFront = !shooterFront;
    }
    if(!shooterFront){
      speed = -speed;
    }
    
    }
    if(!shooterFront){
      LEDLights.pattern = 4;
    }
    else{
      LEDLights.pattern = 3;
      relay.set(false);
    }
  
    move();
  }

  
  public void autoRun() {
    //System.out.println(speed);
    autoOrient();
    move();
  }
// F I X   T H I S
  public void altAutoRun(double startTime, double endTime, double driveSpeed, double driveRotation){
    if(Robot.timer.get() > startTime && Robot.timer.get() < endTime){
      speed = -driveSpeed;
      rotation = driveRotation;
      move();
    }
   /* else {
      speed = 0;
      rotation = 0;
      move();
    }
    */
  }
}