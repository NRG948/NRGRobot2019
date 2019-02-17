package frc.robot.commandGroups;

import static frc.robot.subsystems.HatchClawSubsystem.State.CLOSE;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.EXTEND;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.RETRACT;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;

public class DeliverHatch extends CommandGroup {
  private static final double HATCH_EXTEND_DELAY = 0.1;
  /**
   * Command group for Delivering a hatch cover after the robot's in the right position.
   */
  public DeliverHatch() {
    addSequential(new HatchExtension(EXTEND));
    addSequential(new DelaySeconds(HATCH_EXTEND_DELAY));
    addSequential(new HatchClaw(CLOSE));
    addSequential(new HatchExtension(RETRACT));
  }
}
