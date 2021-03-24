package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VisionLight;

public class CameraSetDriveSetpoint extends Command {
    private static final int PERIOD = 10; //period in milliseconds
    private static final double TIME_FOR_RELAY_TO_SWITCH = 0.033; // Most relays are 0.005s to 0.020s
    private static final double TIME_FOR_PI_TO_GET = 0.167; //If the pi has 30 frames/second, this is 5 frames

    private Drive drive;
    private VisionLight light;
    private boolean finished;
    public CameraSetDriveSetpoint(String name) {
        super(name, PERIOD, false);
        this.drive = Robot.drive;
        this.light = Robot.light;
        finished = false;
    }
    @Override
    protected void onStart() {
        light.lock();
        light.turnOn();
    }
    @Override
    protected void onFinish() {
        light.turnOff();
        light.unlock();
    }
    @Override
    protected void whileRunning() {
        if(getTime() > TIME_FOR_RELAY_TO_SWITCH + TIME_FOR_PI_TO_GET) {
            drive.cameraOrient();
            finished = true;
        }
    }
    @Override
    protected boolean isFinished() {
        return finished;
    }
}