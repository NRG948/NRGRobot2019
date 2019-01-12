/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int DRIVE_FRONT_LEFT_MOTOR_PORT = 0;
  public static final int DRIVE_FRONT_RIGHT_MOTOR_PORT = 1;
  public static final int DRIVE_BACK_LEFT_MOTOR_PORT = 2;
  public static final int DRIVE_BACK_RIGHT_MOTOR_PORT = 3;

  public static SpeedController driveFrontLeftMotor;
  public static SpeedController driveFrontRightMotor;
  public static SpeedController driveBackLeftMotor;
  public static SpeedController driveBackRightMotor;

  public static void init() {
    driveFrontLeftMotor = new Talon(DRIVE_FRONT_LEFT_MOTOR_PORT);
    driveFrontRightMotor = new Talon(DRIVE_FRONT_RIGHT_MOTOR_PORT);
    driveBackLeftMotor = new Talon(DRIVE_FRONT_LEFT_MOTOR_PORT);
    driveBackRightMotor = new Talon(DRIVE_FRONT_RIGHT_MOTOR_PORT);
    
    driveFrontRightMotor.setInverted(true);
    driveBackRightMotor.setInverted(true);
  }

  
}


