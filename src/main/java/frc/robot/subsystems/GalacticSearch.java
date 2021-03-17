package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.AutonomousDrive;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;




public class GalacticSearch{

private SpeedControllerGroup left,right;
public DifferentialDrive drive;
private Ultrasonic ultrasonic;
public DigitalInput sensor; 
PWMVictorSPX lowerConveyor,upperConveyor;



boolean coarseAngleFound, fineAngleFound, ballFound, startGalactic, startTurn, cameraCentered, ballLocated, intakeInProgress, finishedDriving, ballDetected;
Servo servo;
int sweepDirection, count, ballCount;
double offsetAngle, servoAngle, initialPosition, rotationAngle, distance, speedLow, speedHigh;
private static Pixy2 pixy;
public static int blockCount;
static int sensorCount;



public  void initialize() {
    pixy = Pixy2.createInstance(new SPILink()); // Creates a new Pixy2 camera using SPILink
    pixy.init(); // Initializes the camera and prepares to send/receive data
    pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
    pixy.setLED(255, 255, 255); // Sets the RGB LED to full white
}



public GalacticSearch(){

left = new SpeedControllerGroup(RobotMap.leftFront, RobotMap.leftBack);
right = new SpeedControllerGroup(RobotMap.rightFront, RobotMap.rightBack);
ultrasonic = RobotMap.ultrasonic;

coarseAngleFound = false;
fineAngleFound = false;
ballFound = false;
startTurn = false;
cameraCentered = false;
ballLocated = false;
intakeInProgress = false;
finishedDriving = false;
ballDetected = false;


speedLow = 0.6;
speedHigh = 0.8;
ballCount = 0;
 


sweepDirection = 1;
initialPosition = 0;
rotationAngle = 0.0;
servo = RobotMap.pixyServo;
blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);

startGalactic = MechanismsJoystick.startGalacticSearch();
offsetAngle = 0;
servoAngle = 0;
}
//this Mode makes the servo sweep around to detect a ball
public static Block getBiggestBlock() {
    // Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
    // does not wait for new data if none is available,
    // and limits the number of returned blocks to 25, for a slight increase in efficiency
    
    
    if (blockCount <= 0) {
        return null; // If blocks were not found, stop processing
    }
    ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
    Block largestBlock = null;
    for (Block block : blocks) { // Loops through all blocks and finds the widest one
        if (largestBlock == null) {
            largestBlock = block;
        } else if (block.getWidth() > largestBlock.getWidth()) {
            largestBlock = block;
        }
    }
    return largestBlock;
}


public void modeOne(){
    
        blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
        if (blockCount >= 1) { 
            ballFound = true;
            offsetAngle = getBiggestBlock().getAngle() + servo.getAngle(); //offset of ball from robot center = angle of ball from pixy center + angle of servo from robot center
        

    if (ballFound == false) {
        //Sweeps servo to find ball
        if (sweepDirection == 1) {
            servo.setAngle(servo.getAngle()+1.4);
        }
        if (sweepDirection == 0) {
            servo.setAngle(servo.getAngle()-1.4);
        }
        if (servo.getAngle()>175) {
            sweepDirection = 0;
        }
        if (servo.getAngle()<5) {
            sweepDirection = 1;
        }
        
    }
    
}


}

//Drive rotation mode
 public void modeTwo(double rotationAngle) {

    //if servoAngle is less than 0, then turn left
    
    if(!startTurn){
        AutonomousDrive.Turn(rotationAngle);
        startTurn = true;
    }
    servo.setAngle(0);
    
}
//pixy checks if ball is in center of camera
public double modeThree(){
//get pixy offset x in pixels to degrees
//return the offset 
servo.setAngle(0);
double offsetAngle = getBiggestBlock().getAngle(); //stealing from step 1
return offsetAngle;
/*
else if (getBiggestBlock().getY() >= -5 && getBiggestBlock().getX() <= 5){
    modeFour();
}
*/
}


//Mode Four drives robot forward x amount of distance
public void modeFour(){
    //distance is determined by ultrasonic

    double range = ultrasonic.getRangeInches();
    AutonomousDrive.DriveByDistance(range);
    if(range <= 0){
        finishedDriving = true;
    }
    

    

}
//drive forward and pass in a distance in inches
//receive info if robot has gone that distance
//if so, set booleans to false to repeat code


//Mode 5 intakes the ball
public void modeFive(){
    if (isBallFound()){
        //run intake system
        lowerConveyor.set(-speedHigh);
        upperConveyor.set(speedHigh);
        intakeInProgress = true;
        ballDetected = true;
 
    }
   else //stop intake system
        lowerConveyor.set(0);
        upperConveyor.set(0);
        intakeInProgress = false;
        ballDetected = false;
 
// if ball intake progress and ball not found then ballCount2
if ((intakeInProgress == true) && (!isBallFound())){
 
    ballCount ++;
 
}else{
// ball not found and stop intake
    ballCount = 0;
   lowerConveyor.set(0);
   upperConveyor.set(0);
   intakeInProgress = false;
    }
 
// if ball count greater than 3 go mode 6

}
 
 
public boolean isBallFound(){
 
if (!sensor.get()){
    sensorCount ++;
    }
 
    if (sensor.get()){
        sensorCount = 0;
 
    }
 
    if (sensorCount >= 3){
        
    return true;
 
    }
 
     else return false;   
}
 
 


//mode six drives to the end zone
public void modeSix(){
//robot needs to turn towards the end zone 
distance = ultrasonic.getRangeInches();
AutonomousDrive.DriveByDistance(distance);
if(distance <= 0){
left.stopMotor();
right.stopMotor();
}
}


public void run(){

boolean done = false;
if(startGalactic){
  while(!done){  
    if(!ballFound){
      modeOne();

    }

if(ballFound && !coarseAngleFound){

    modeTwo(servoAngle);
    coarseAngleFound = true;
    startTurn = false;
    
}
if(coarseAngleFound && !fineAngleFound){
   double offset = modeThree();
   if (offsetAngle <= -30 || offsetAngle <= 30){
    modeOne();

}
else if (offsetAngle >= -29 && offsetAngle <= 29){
    double rotationAngle = offsetAngle;
    modeTwo(rotationAngle);


}
   modeTwo(offset);
   fineAngleFound = true;
   startTurn = false;
}

if(fineAngleFound){
    modeFour();
    
}
modeFive();
if( (finishedDriving && !ballDetected ) || ( ballDetected && ballCount < 3) ){
   ballFound = false;
   coarseAngleFound = false;
   startTurn = false;
   fineAngleFound = false;
   
   modeOne();
}

else if(count >= 3){
    done = true;
    modeSix();
}


}


}



}








}