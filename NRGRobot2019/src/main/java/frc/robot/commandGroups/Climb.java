/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ClimberPistons;
public class Climb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Climb() {
    addSequential(new ClimberPistons(true));
    // move the motor a certain distance
    addSequential(new ClimberPistons(false));
    // move the motor a certain distance
  }
}
