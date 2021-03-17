package frc.robot.subsystems;
 
public class AutoNav {
    public AutonomousDrive drive;
 
    public AutoNav() {
        drive = new AutonomousDrive();
    }
 
    public void forward(double inches){
        drive.DriveByDistance(inches);
    }
    
    public void turn(double degrees){
        drive.Turn(degrees);
    }
    
    public void barrelRacing() {
        forward(138);
        for (int i = 0; i <= 3; i++) {
            turn(90);
            forward(60);
        }
        turn(90);
        forward(150);
        for (int i = 0; i<= 3; i++) {
            turn(-90);
            forward(60);
        }
        turn(-45);
        forward(85.2);
        turn(45);
        for(int i = 0; i<=2; i++) {
            turn(-90);
            forward(60);
        }
        turn(-90);
        forward(264);
    }
 
    public void bounce() {
        forward(48);
        turn(-90);
        forward(63);
        forward(-63);
        turn(90);
        forward(30);
        turn(90);
        forward(60);
        turn(-90);
        forward(60);
        turn(-90);
        forward(123);
        forward(-123);
        turn(90);
        forward(90);
        turn(-90);
        forward(123);
        forward(-63);
        turn(-90);
        forward(-60);
        turn(360);
    }
 
    public void slalom() {
        forward(48);
        turn(-90);
        forward(60);
        turn(90);
        forward(180);
        turn(90);
        forward(60);
        turn(-90);
        forward(60);
        turn(-90);
        forward(60);
        turn(90);
        forward(-60);
        turn(90);
        forward(-180);
        turn(90);
        forward(-60);
        turn(90);
        forward(-60);
        turn(1080);
    }
}
