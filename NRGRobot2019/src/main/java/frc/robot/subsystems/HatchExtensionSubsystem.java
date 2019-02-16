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
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;

/**
 * Add your docs here.
 */
public class HatchExtensionSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum State{
    EXTEND, RETRACT;
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchExtension(State.RETRACT));
  }

  public void extend(){
    // RobotMap.gearboxSolenoid.set(Value.kForward);
  }

  public void retract(){
    // RobotMap.gearboxSolenoid.set(Value.kReverse);
  }
}
