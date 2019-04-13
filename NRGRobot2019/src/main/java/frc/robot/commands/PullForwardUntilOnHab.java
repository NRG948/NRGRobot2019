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

public class PullForwardUntilOnHab extends Command {
  private static final double OVER_HAB_THRESHOLD = 0.6;
  private final boolean level3;

  public PullForwardUntilOnHab(boolean level3) {
    requires(Robot.climberArmWheels);
    this.level3 = level3;
    if (this.level3) {
      requires(Robot.cargoAcquirer);
      requires(Robot.drive);
    }
  }

  public PullForwardUntilOnHab() {
    this(false);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("PullForwardUntilOnHab init Ir: " + RobotMap.IRSensor.getAverageVoltage());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double power = NumberPrefs.CLIMBER_ARM_WHEELS_POWER.getValue();
    Robot.climberArmWheels.spin(power);
    if (this.level3) {
      // Robot.cargoAcquirer.acquire(0.2, Direction.ACQUIRE);
      if (RobotMap.climberRearEncoder.getDistance() >= 2400) {
        Robot.drive.tankDrive(0.3, 0.3, false);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (RobotMap.IRSensor.getAverageVoltage() > OVER_HAB_THRESHOLD)
        && (RobotMap.climberRearEncoder.getDistance() >= NumberPrefs.CLIMBER_REAR_MIN_TICKS.getValue());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climberArmWheels.stop();
    if (this.level3) {
      Robot.cargoAcquirer.stop();
      Robot.drive.stopMotor();
    }
    System.out.println("PullForwardUntilOnHab end Ir: " + RobotMap.IRSensor.getAverageVoltage() + " ClimbTicks: "
        + RobotMap.climberRearEncoder.getDistance());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
