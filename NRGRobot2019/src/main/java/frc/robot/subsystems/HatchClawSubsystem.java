/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.HatchClaw;

/**
 * Add your docs here.
 */
public class HatchClawSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum State{
    OPEN, CLOSE, RETRACT;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchClaw(State.OPEN));
  }

  public void setClawOpen(){
    // RobotMap.hatchClawSolenoid.set(Value.kForward);
  }

  public void setClawClose(){
    // RobotMap.hatchClawSolenoid.set(Value.kReverse);
  }
}
