/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.GearShift;

/**
 * Add your docs here.
 */
public class Gearbox extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum Gear {
    HIGH, LOW;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GearShift(Gear.LOW));
  }

  public void setHighGear(){
    // RobotMap.gearboxSolenoid.set(Value.kForward);
  }

  public void setLowGear(){
    // RobotMap.gearboxSolenoid.set(Value.kReverse);
  }
}
