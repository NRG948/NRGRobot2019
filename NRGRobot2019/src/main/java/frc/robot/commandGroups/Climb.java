package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.ClimberMotor;
import frc.robot.commands.ActivateClimberPistons;
public class Climb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Climb() {
    addSequential(new ActivateClimberPistons(true));
    // move the motor a certain distance
    addSequential(new ActivateClimberPistons(false));
    // move the motor a certain distance
  }
}
