/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Controls.MechanismsJoystick;

public class IntakeTest {
    public static double HOLD_SPEED = 0.15;
    public static double MOVE_SPEED = 1;
    public static double INTAKE_SPEED = 0.6;
    public static double AVOID_INTAKING_SPEED = -0.4;

    private PWMVictorSPX motor, armMotorOne, armMotorTwo;
    private boolean out;

    public IntakeTest(){
        motor = RobotMap.intakeMotor;
        motor.setInverted(true);
        armMotorOne = RobotMap.armMotorLeft;
        armMotorTwo = RobotMap.armMotorRight;
        out = false;
    }


    public void run(){

        if(MechanismsJoystick.getEject()){
            motor.set(-INTAKE_SPEED);
        }
        else if(MechanismsJoystick.getToggleIntakeMotor()) {
            motor.set(INTAKE_SPEED);
        }
        else {
            motor.set(AVOID_INTAKING_SPEED);
        }

        if(MechanismsJoystick.isManual()){
            SmartDashboard.putBoolean("out", out);
            SmartDashboard.putNumber("left arm motor output", armMotorOne.get());
            SmartDashboard.putNumber("right arm motor output", armMotorTwo.get());
            if(MechanismsJoystick.getToggleManArmMotor()) {
                out = !out;
            }
            if(!out){
                if(MechanismsJoystick.getToggleArmMotor()) {
                    armMotorOne.set(MOVE_SPEED);
                    armMotorTwo.set(MOVE_SPEED);
                }
                else {
                    armMotorOne.set(HOLD_SPEED*0.75);
                    armMotorTwo.set(HOLD_SPEED);
                }
            }
            else {
                if(MechanismsJoystick.getToggleArmMotor()) {
                    armMotorOne.set(-MOVE_SPEED);
                    armMotorTwo.set(-MOVE_SPEED);
                }
                else {
                    armMotorOne.set(-HOLD_SPEED*0.75);
                    armMotorTwo.set(-HOLD_SPEED);
                }
            }
        }
    }

}
