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
import frc.robot.utilities.NRGPreferences.NumberPrefs;

public class SetClimberHeight extends Command {
  private double height;
  private double initialError;

  public SetClimberHeight(double height) {
    requires(Robot.climberRear);
    this.height = height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("SetClimberHeight init height: " + this.height);
    initialError = height - RobotMap.climberRearEncoder.getDistance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.climberRear.rawClimb(Math.signum(initialError) * NumberPrefs.CLIMBER_REAR_POWER.getValue());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double currentDistance = RobotMap.climberRearEncoder.getDistance();
    return initialError < 0 ? currentDistance >= this.height: currentDistance <= this.height;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("SetClimberHeight end currentHeight: " + RobotMap.climberRearEncoder.getDistance());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
