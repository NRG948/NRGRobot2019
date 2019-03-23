/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import static frc.robot.subsystems.HatchClawSubsystem.State.CLOSE;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.EXTEND;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.RETRACT;
import static frc.robot.subsystems.HatchExtensionSubsystem.HATCH_EXTEND_DELAY;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveToVisionTape;
import frc.robot.commands.DriveToVisionTapeThree;
import frc.robot.commands.DriveToVisionTapeTwo;
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;
import frc.robot.subsystems.HatchClawSubsystem.State;
import frc.robot.utilities.VisionTargetsApproach;

public class PickupHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickupHatch() {
    addSequential(new HatchClaw(State.CLOSE));
    addSequential(new DriveToVisionTapeThree(VisionTargetsApproach.HatchPickUp));
    addSequential(new DelaySeconds(HATCH_EXTEND_DELAY));
    addSequential(new HatchExtension(EXTEND));
    addSequential(new DelaySeconds(HATCH_EXTEND_DELAY));
    addSequential(new HatchClaw(State.OPEN));
    addSequential(new DelaySeconds(HATCH_EXTEND_DELAY));
    addSequential(new HatchExtension(RETRACT));
  }
}
