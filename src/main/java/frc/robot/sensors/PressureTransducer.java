package frc.robot.sensors;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

public class PressureTransducer {
    private AnalogInput sensor;
    public static final int VALUE_AT_120 = 2938;
    public static final int VALUE_AT_0 = 425;
    public PressureTransducer() {
        sensor = RobotMap.pressureTransducer;
    }
    public double getPSI() {
        return (sensor.getValue() - 425)/20.94;
    }
}