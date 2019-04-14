package frc.robot.commandGroups;

import static frc.robot.subsystems.HatchClawSubsystem.State.CLOSE;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.EXTEND;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.RETRACT;
import static frc.robot.subsystems.HatchExtensionSubsystem.HATCH_EXTEND_DELAY;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.DriveToVisionTape;
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;
import frc.robot.commands.WaitForNewVisionData;
import frc.robot.subsystems.HatchClawSubsystem.State;
import frc.robot.utilities.VisionTargetsApproach;

public class DeliverHatch extends CommandGroup {
  /**
   * Command group for Delivering a hatch cover after the robot's in the right
   * position.
   */
  public DeliverHatch() {
    addSequential(new WaitForNewVisionData());
    addSequential(new DriveToVisionTape(VisionTargetsApproach.HatchDeliver), 2.0);
    addParallel(new HatchClaw(CLOSE));
    addSequential(new HatchExtension(EXTEND));
    addSequential(new DelaySeconds(HATCH_EXTEND_DELAY));
    addSequential(new HatchExtension(RETRACT));
    addSequential(new DriveStraightDistance(6, -0.8));
    addSequential(new HatchClaw(State.OPEN));
  }
}
