/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
/*
  public static final int DRIVE_FRONT_LEFT_MOTOR_PORT = 2;
  public static final int DRIVE_FRONT_RIGHT_MOTOR_PORT = 0;
  public static final int DRIVE_BACK_LEFT_MOTOR_PORT = 3;
  public static final int DRIVE_BACK_RIGHT_MOTOR_PORT = 1;
  */
   public static final int DRIVE_LEFT_ENCODER_PORT1 = 0; 
  public static final int DRIVE_LEFT_ENCODER_PORT2 = 1;
  public static final int DRIVE_RIGHT_ENCODER_PORT1 = 2;
  public static final int DRIVE_RIGHT_ENCODER_PORT2 = 3;
  
  public static final double DRIVE_LEFT_ENCODER_DIST_PER_PULSE = 0.01171875;
  public static final double DRIVE_RIGHT_ENCODER_DIST_PER_PULSE = 0.01171875;

  /*
  public static SpeedController driveFrontLeftMotor;
  public static SpeedController driveFrontRightMotor;
  public static SpeedController driveBackLeftMotor;
  public static SpeedController driveBackRightMotor;
*/
  public static WPI_VictorSPX driveFrontLeftMotor;
  public static WPI_VictorSPX driveMiddleLeftMotor;
  public static WPI_VictorSPX driveBackLeftMotor;
  public static WPI_VictorSPX driveFrontRightMotor;
  public static WPI_VictorSPX driveMiddleRightMotor;
  public static WPI_VictorSPX driveBackRightMotor;

  public static Encoder driveLeftEncoder;
  public static Encoder driveRightEncoder;

  public static AHRS navx;

  public static void init() {
    driveFrontLeftMotor = new WPI_VictorSPX(5);
    driveBackLeftMotor = new WPI_VictorSPX(6);
    driveMiddleLeftMotor = new WPI_VictorSPX(1);
    driveMiddleRightMotor = new WPI_VictorSPX(3);
    driveFrontRightMotor = new WPI_VictorSPX(4);
    driveBackRightMotor = new WPI_VictorSPX(2);

    driveFrontRightMotor.setInverted(true);
    driveFrontLeftMotor.setInverted(true);
    driveMiddleLeftMotor.setInverted(true);
    driveMiddleRightMotor.setInverted(true);
    driveBackLeftMotor.setInverted(true);
    driveBackRightMotor.setInverted(true);

    driveLeftEncoder = new Encoder(DRIVE_LEFT_ENCODER_PORT1, DRIVE_LEFT_ENCODER_PORT2, true);
    driveRightEncoder = new Encoder(DRIVE_RIGHT_ENCODER_PORT1, DRIVE_RIGHT_ENCODER_PORT2);
    driveLeftEncoder.setDistancePerPulse(DRIVE_LEFT_ENCODER_DIST_PER_PULSE);
    driveRightEncoder.setDistancePerPulse(DRIVE_RIGHT_ENCODER_DIST_PER_PULSE);

    navx = new AHRS(SPI.Port.kMXP); 

  }

  public static void resetSensors(){
    driveLeftEncoder.reset();
    driveRightEncoder.reset();
    navx.reset();
    System.out.println("Sensors Reset");
  }
  
}


