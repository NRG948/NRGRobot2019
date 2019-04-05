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
import frc.robot.subsystems.CargoAcquirer.Direction;
import frc.robot.utilities.NRGPreferences.NumberPrefs;

public class PullForwardUntilOnHab extends Command {
  private static final double OVER_HAB_THRESHOLD = 0.6;
  public PullForwardUntilOnHab() {
    requires(Robot.climberArmWheels);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("PullForwardUntilOnHab init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double power = NumberPrefs.CLIMBER_ARM_WHEELS_POWER.getValue();
    Robot.climberArmWheels.spin(power);
    // Robot.cargoAcquirer.acquire(0.2, Direction.ACQUIRE);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotMap.IRSensor.getAverageVoltage() > OVER_HAB_THRESHOLD;
}

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climberArmWheels.stop();
    System.out.println("PullForwardUntilOnHab end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
