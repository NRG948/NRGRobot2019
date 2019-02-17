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
