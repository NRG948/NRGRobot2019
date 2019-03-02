package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.ClimberRear;
import frc.robot.commands.ActivateClimberPistons;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveStraightDistance;
public class Climb extends CommandGroup {
  /**
   * Perform a level three climbing sequence.
   */
  public Climb() {
    addSequential(new DriveStraightDistance(10, -0.3));
    addSequential(new ActivateClimberPistons(true));
    addSequential(new DelaySeconds(1.0));
    // move the motor a certain distance
    addSequential(new ActivateClimberPistons(false));
    // move the motor a certain distance
  }
}
