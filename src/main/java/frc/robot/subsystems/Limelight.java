package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;




public class Limelight {

boolean isTracking;
double x, y, area;
NetworkTable table, tx, ty,ta;
private NetworkTableEntry ledMode;
private NetworkTableEntry camMode;

public Limelight(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
    
    //read values periodically
     x = tx.getDouble(0.0);
     y = ty.getDouble(0.0);
     area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    isTracking = false;
}
public boolean getTargetFound() {
    NetworkTableEntry tv = table.getEntry("tv");
    double v = tv.getDouble(0);
    if (v == 0.0){
        return false;
    }else {
        return true;
    }
}
public void trackingMode(){
    camMode.setDouble(0);
    ledMode.setDouble(0);
    isTracking = true;
}



public void calibrateLimelight(){
    trackingMode();
    if(getTargetFound() & isTracking == true){

        if(x < -0.5 && x > 0.5){

            if(y < -0.5 && y > 0.5){
            //adjust da shooters
            isTracking = true;
        }

        else{
            //leave the shooters in a default form
        }
    }
    else{
        trackingMode();
        isTracking = false;
    }
    
}


}
}