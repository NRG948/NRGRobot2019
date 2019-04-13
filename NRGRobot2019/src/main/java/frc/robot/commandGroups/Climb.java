package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ActivateClimberPistons;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.PullForwardUntilOnHab;
import frc.robot.commands.SetClimberHeight;
import frc.robot.commands.SetRobotRoll;
import frc.robot.subsystems.ClimberPistons.State;

public class Climb extends CommandGroup {
  /**
   * Perform a level three climbing sequence.
   */
  public Climb() {
    addSequential(new ActivateClimberPistons(State.EXTEND));
    addSequential(new DelaySeconds(0.5));
    addSequential(new SetRobotRoll(8.0));
    addSequential(new PullForwardUntilOnHab());
    addSequential(new SetClimberHeight(75));
    addParallel(new DriveStraightDistance(6, 0.3), 1.0);
    addSequential(new ActivateClimberPistons(State.RETRACT));
  }
}
