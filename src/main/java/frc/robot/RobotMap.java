package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;

public class RobotMap {
    public static final Joystick driveJoystick = new Joystick(0);
    public static final Joystick mechanismsJoystick = new Joystick(1);


    public static final CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    public static final CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    public static final CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    public static final CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);


    public static final CANEncoder leftFrontEncoder = new CANEncoder(leftFront, EncoderType.kQuadrature, 42);
    public static final CANEncoder rightFrontEncoder = new CANEncoder(rightFront, EncoderType.kQuadrature, 42);
    public static final CANEncoder leftBackEncoder = new CANEncoder(leftBack, EncoderType.kQuadrature, 42);
    public static final CANEncoder rightBackEncoder = new CANEncoder(rightBack, EncoderType.kQuadrature, 42);

    public static final Servo pixyServo = new Servo(3); //needs actual port number
    
    public static final Ultrasonic ultrasonic = new Ultrasonic(1,2); //needs a home in the form of a port number

}