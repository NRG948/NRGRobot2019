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
import frc.robot.utilities.NRGPreferences.NumberPrefs;

public class SetRobotRoll extends Command {
  private double roll;

  public SetRobotRoll(double roll) {
    requires(Robot.climberRear);
    this.roll = roll;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("SetRobotPitch init pitch: " + this.roll);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double error = this.roll - RobotMap.navx.getRoll();
    double maxPower = NumberPrefs.CLIMBER_REAR_POWER.getValue();
    double power = MathUtil.clamp(error * 0.5, -maxPower, maxPower);
    Robot.climberRear.rawClimb(-power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(this.roll - RobotMap.navx.getRoll()) <= 1.0)
        && (RobotMap.climberRearEncoder.getDistance() >= NumberPrefs.CLIMBER_REAR_MIN_TICKS.getValue());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climberRear.stop();
    System.out.println("SetRobotPitch end current roll: " + RobotMap.navx.getRoll());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
