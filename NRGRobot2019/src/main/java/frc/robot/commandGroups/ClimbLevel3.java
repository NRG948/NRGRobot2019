/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.ClimberRear;
import frc.robot.subsystems.ClimberPistons.State;
import frc.robot.commands.ActivateClimberPistons;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.PullForwardUntilOnHab;
import frc.robot.commands.SetClimberHeight;
import frc.robot.commands.SetRobotRoll;

public class ClimbLevel3 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbLevel3() {
    addSequential(new ActivateClimberPistons(State.EXTEND));
    addSequential(new DelaySeconds(0.5));
    addParallel(new SetRobotRoll(2.0));
    addSequential(new PullForwardUntilOnHab(true));
    addSequential(new SetClimberHeight(75));
    addParallel(new DriveStraightDistance(6, 0.3), 1.0);
    addSequential(new ActivateClimberPistons(State.RETRACT));
  }
}
