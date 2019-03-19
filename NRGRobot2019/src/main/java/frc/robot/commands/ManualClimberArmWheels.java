/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utilities.MathUtil;

public class ManualClimberArmWheels extends Command {
  public ManualClimberArmWheels() {
    requires(Robot.climberArmWheels);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("ManualClimberArmWeels init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.climberArmsButton.get()) {
      double speed = Robot.oi.getXboxRightY();
      Robot.climberArmWheels.spin(MathUtil.deadband(speed, 0.1));
    } else {
      Robot.climberRear.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.climberArmLeftWheelMotor.set(0);
    RobotMap.climberArmRightWheelMotor.set(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
