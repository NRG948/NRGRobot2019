/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualDrive extends Command {
  public ManualDrive() {
    requires(Robot.drive);// this uses drive subsystem 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Manual Drive Init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double left = Robot.oi.getLeftJoystickY();
    double right = Robot.oi.getRightJoystickY();
    Robot.drive.tankDrive(left, right);
  }// Since this command is used to drive the robot manually using joysticks this method gets the values from the X joysticks and the Y sticks 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }// this command should never be terminated because we need this in order to drive the robot 

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stopMotor();
  } // stops the robots motor 

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
