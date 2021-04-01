/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AutoNav;
import frc.robot.subsystems.AutonomousDrive;
import frc.robot.subsystems.CameraServo;
import frc.robot.subsystems.ConveyorBelt;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.GalacticSearch;
import frc.robot.subsystems.IntakeTest;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.LIDAR;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLight;
import io.github.pseudoresonance.pixy2api.Pixy2;
import frc.robot.Controls.MechanismsJoystick;
import frc.robot.sensors.PressureTransducer;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static AutonomousDrive AutoDrive;
  public static GalacticSearch galacticsearch;
  public static AutoNav autoNav;
  public static Subsystem subsystems[];
  public static Drive drive;
  public static Shooter shooter;
  public static CameraServo cameraServo;
  public static ConveyorBelt conveyor;
  public static SmartDashboardOutputs outputs;
  public static IntakeTest intake;
  public static VisionLight light;
  public static PressureTransducer pressureTransducer;
  public static Timer timer;
  public static LEDLights led;
  public static Pixy2 pixycam;
  public static LIDAR lidar;
  
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    AutoDrive = new AutonomousDrive();
    galacticsearch = new GalacticSearch();
    autoNav = new AutoNav();
    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    drive = new Drive();
    lidar = new LIDAR();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    galacticsearch.run();
    if(MechanismsJoystick.autoNavOne()){
      autoNav.barrelRacing();
    }
    if(MechanismsJoystick.autoNavTwo()){
      autoNav.bounce();
    }
    if(MechanismsJoystick.autoNavOne() && MechanismsJoystick.autoNavTwo()){
      autoNav.slalom();
    }
    lidar.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    RobotMap.lightRelay.set(true);

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    AutoDrive.printState();
    galacticsearch.run();
    drive.run();
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
