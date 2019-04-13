/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualClimberArmWheels;

/**
 * Add your docs here.
 */
public class ClimberArmWheels extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualClimberArmWheels());
  }

  public void spin(double power) {
    power = Math.abs(power);
    RobotMap.climberArmLeftWheelMotor.set(power);
    RobotMap.climberArmRightWheelMotor.set(power);
  }

  public void stop() {
  }
}
