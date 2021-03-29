package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class RobotMap {
    public static final Joystick driveJoystick = new Joystick(0);
    public static final Joystick mechanismsJoystick = new Joystick(1);

    public static final AHRS ahrs = new AHRS();

    public static final CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
    public static final CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
    public static final CANSparkMax rightFront = new CANSparkMax(3, MotorType.kBrushless);
    public static final CANSparkMax rightBack = new CANSparkMax(4, MotorType.kBrushless);
    public static final PWMVictorSPX controlPanelMotor = new PWMVictorSPX(0);
    public static final PWMVictorSPX upperConveyorMotor = new PWMVictorSPX(1);
    public static final PWMVictorSPX lowerConveyorMotor = new PWMVictorSPX(2);
    public static final PWMVictorSPX intakeMotor = new PWMVictorSPX(3);
    public static final PWMVictorSPX armMotorLeft = new PWMVictorSPX(4);
    public static final PWMVictorSPX armMotorRight = new PWMVictorSPX(5);


    public static final CANEncoder leftFrontEncoder = new CANEncoder(leftFront, EncoderType.kQuadrature, 42);
    public static final CANEncoder rightFrontEncoder = new CANEncoder(rightFront, EncoderType.kQuadrature, 42);
    public static final CANEncoder leftBackEncoder = new CANEncoder(leftBack, EncoderType.kQuadrature, 42);
    public static final CANEncoder rightBackEncoder = new CANEncoder(rightBack, EncoderType.kQuadrature, 42);

    public static final Servo pixyServo = new Servo(8); //needs actual port number

    public static final DigitalOutput lightRelay = new DigitalOutput(9);

    public static final AnalogInput pixy = new AnalogInput(2);
    
    public static final Ultrasonic ultrasonic = new Ultrasonic(4,5); //needs a home in the form of a port number
    public static final Servo camera = new Servo(7);
    public static final DigitalInput intakeSensorOne = new DigitalInput(2);
    public static final DigitalInput shooterSensor = new DigitalInput(3);
    public static final AnalogInput pressureTransducer = new AnalogInput(3);

    public static final String shooterCameraName = "Microsoft LifeCam HD-3000";
    public static final Relay visionLightRelay = new Relay(0);

    public static final TalonSRX shooterTopMotor = new TalonSRX(5);
    public static final TalonSRX shooterBottomMotor = new TalonSRX(6);

    public static final AddressableLED led = new AddressableLED(9);
    
}