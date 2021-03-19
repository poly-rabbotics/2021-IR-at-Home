//<= dunno why error
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter {

    private TalonSRX topMotor, bottomMotor;
    private double topMotorSpeed, bottomMotorSpeed;
    public Shooter() {
        topMotorSpeed = 0;
        bottomMotorSpeed = 0;
        topMotor = RobotMap.shooterTopMotor;
        bottomMotor = RobotMap.shooterBottomMotor; 
        //distance = 0.0;
        //preset = 1;
        //lowSpeed = 0.225;
        //highSpeed = 0.525;
        //bothSpeed = 0.7;
        //solenoid = RobotMap.shooterSolenoid;
        //solenoidOut = false;
        //configureShooterPids();
    }
    public void adjustSpeeds(){
        if(MechanismsJoystick.getChangeTopShooter() > 0.1 && topMotorSpeed <= 1) {
          topMotorSpeed += .005;
        }
        else if(MechanismsJoystick.getChangeTopShooter() < -0.1 && topMotorSpeed >= 0) {
          topMotorSpeed -= .005;
        }
        if(MechanismsJoystick.getChangeBottomShooter() > .1 && bottomMotorSpeed <= 1) {
          bottomMotorSpeed -= .005;
        }
        else if(MechanismsJoystick.getChangeBottomShooter() <- .1 && bottomMotorSpeed >= 0) {
          bottomMotorSpeed -= .005;
        }
    }

    
    double topMotorV = topMotorSpeed * 2048 / 600;
    double bottomMotorV = bottomMotorSpeed * 2048 / 600;

    public void calibrateShooter(){
  
      if (MechanismsJoystick.getToggleManShootOne()){
        bottomMotorSpeed += 10;
      }
      if (MechanismsJoystick.getToggleManShootTwo()){
        topMotorSpeed -= 10;
      }
  
      if (MechanismsJoystick.getToggleManShootThree()){
        topMotorSpeed += 10;
        bottomMotorSpeed -= 10;
      }
    }
    

    public void run() {
 
        if(MechanismsJoystick.allowCalibrateShooter()){
            calibrateShooter();
          }
          else{
            
            if(MechanismsJoystick.isManual()) {
                adjustSpeeds();
                SmartDashboard.putNumber("Top Shooter:", topMotorSpeed);
                SmartDashboard.putNumber("Bottom Shooter:", bottomMotorSpeed);
                RobotMap.shooterTopMotor.set(ControlMode.PercentOutput, topMotorSpeed);
                RobotMap.shooterBottomMotor.set(ControlMode.PercentOutput, bottomMotorSpeed);
                /*if(MechanismsJoystick.getToggleManShootThree()){
                  presetThree();
                }
                else if(MechanismsJoystick.getToggleManShootTwo()){
                  presetTwo();
                }
                else if(MechanismsJoystick.getToggleManShootOne()){
                  presetOne();
                }
                else{
                  shooterTopMotor.set(ControlMode.PercentOutput, 0);
                  shooterBottomMotor.set(ControlMode.PercentOutput, 0);
                  solenoid.set(Value.kForward);
                }*/
                /*
                if(MechanismsJoystick.getToggleManShooterSolenoid()) {
                  solenoid.set(Value.kReverse);
                }
                else if(!MechanismsJoystick.getToggleManShooterSolenoid()) {
                  solenoid.set(Value.kForward);
                }
                */
    
            }
        }
    }
}