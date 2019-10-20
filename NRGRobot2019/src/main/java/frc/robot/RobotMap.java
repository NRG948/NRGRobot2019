/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static WPI_VictorSPX driveFrontLeftMotor;
  public static WPI_VictorSPX driveMiddleLeftMotor;
  public static WPI_VictorSPX driveBackLeftMotor;
  public static WPI_VictorSPX driveFrontRightMotor;
  public static WPI_VictorSPX driveMiddleRightMotor;
  public static WPI_VictorSPX driveBackRightMotor;
  
  private static final int DRIVE_LEFT_ENCODER_PORT1 = 2;
  private static final int DRIVE_LEFT_ENCODER_PORT2 = 3;
  private static final int DRIVE_RIGHT_ENCODER_PORT1 = 0;
  private static final int DRIVE_RIGHT_ENCODER_PORT2 = 1;
  
  public static final double DRIVE_LEFT_ENCODER_DIST_PER_PULSE = 0.01171875;
  public static final double DRIVE_RIGHT_ENCODER_DIST_PER_PULSE = 0.01171875;

  public static Encoder driveLeftEncoder;
  public static Encoder driveRightEncoder;

  public static AHRS navx;

  public static void init() {
    driveMiddleLeftMotor = new WPI_VictorSPX(1);
    driveBackLeftMotor = new WPI_VictorSPX(2);
    driveFrontLeftMotor = new WPI_VictorSPX(3);
    driveFrontRightMotor = new WPI_VictorSPX(4);
    driveMiddleRightMotor = new WPI_VictorSPX(5);
    driveBackRightMotor = new WPI_VictorSPX(6);

    driveFrontRightMotor.setInverted(true);
    driveFrontLeftMotor.setInverted(true);
    driveMiddleLeftMotor.setInverted(true);
    driveMiddleRightMotor.setInverted(true);
    driveBackLeftMotor.setInverted(true);
    driveBackRightMotor.setInverted(true);

    driveFrontRightMotor.setInverted(true);
    driveFrontLeftMotor.setInverted(true);
    driveBackLeftMotor.setInverted(true);
    driveBackRightMotor.setInverted(true);

    driveLeftEncoder = new Encoder(DRIVE_LEFT_ENCODER_PORT1, DRIVE_LEFT_ENCODER_PORT2);
    driveRightEncoder = new Encoder(DRIVE_RIGHT_ENCODER_PORT1, DRIVE_RIGHT_ENCODER_PORT2, true);
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


