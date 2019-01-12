/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrive;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  private SpeedControllerGroup leftMotor = new SpeedControllerGroup(RobotMap.driveFrontLeftMotor,RobotMap.driveBackLeftMotor);
  private SpeedControllerGroup rightMotor = new SpeedControllerGroup(RobotMap.driveFrontRightMotor, RobotMap.driveBackRightMotor);
  private DifferentialDrive motivator = new DifferentialDrive(leftMotor, rightMotor);
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualDrive());
  }

  public void tankDrive (double leftPower, double rightPower) {
    motivator.tankDrive(leftPower, rightPower);

  }
  public void stopMotor () {
    motivator.stopMotor();
  }
}
