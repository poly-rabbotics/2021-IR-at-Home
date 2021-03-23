package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Controls.DriveJoystick;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionLight extends Subsystem {
    Relay relay;
    public NetworkTableEntry yaw;
    
    public VisionLight() {
        relay = RobotMap.visionLightRelay;
        NetworkTableInstance table = NetworkTableInstance.getDefault();
        NetworkTable cameraTable = table.getTable("Chameleon-Vision").getSubTable("MyCamName");
        yaw = cameraTable.getEntry("yaw");
    }
    public void turnOn() {
        relay.set(Relay.Value.kReverse); //TODO: choose this value such that the lights turn off when reset
    }
    public void turnOff() {
        relay.set(Relay.Value.kForward);
    }
    // haha ...

    public void aim(){
       SmartDashboard.putNumber("yaw", yaw.getDouble(0.0));
    }
   
    public void run() {
        if(DriveJoystick.getToggleLight()) {
            turnOff();
            //Be controlled by manual controls
        }
        else{
            turnOn();
        }
    }
    public void reset() {
        turnOn(); 
        aim();
    }
}