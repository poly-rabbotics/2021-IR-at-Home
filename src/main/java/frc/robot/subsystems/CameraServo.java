package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotMap;

public class CameraServo extends Subsystem {
    Servo camera;
    public CameraServo() {
        camera = RobotMap.camera;
    }
    public void run() {

        if(Drive.shooterFront) {
            camera.set(0.27);
        }
        else {
            camera.set(1.0);
        }

    }
    public void reset() {}
}
