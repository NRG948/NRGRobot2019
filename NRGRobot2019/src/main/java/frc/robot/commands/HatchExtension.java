package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.HatchExtensionSubsystem.State;

/**
 * Command to extend and retract the hatch extender.
 */
public class HatchExtension extends Command {
  private State state;
  public HatchExtension(State state) {
    requires(Robot.hatchExtension);
    this.state = state;
  }

  @Override
  protected void initialize() {
    System.out.println("Hatch Extension " + state);
  }

  @Override
  protected void execute() {
    if(state == State.EXTEND) {
      Robot.hatchExtension.extend();
    } else {
      Robot.hatchExtension.retract();
    }
  }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
    end();
  }
}
