package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.HatchClawSubsystem.State;

/**
 * Command to open and close the hatch claw.
 */
public class HatchClaw extends Command {
  public State state;

  public HatchClaw(State state) {
    requires(Robot.hatchClaw);
    this.state = state;
  }

  @Override
  protected void initialize() {
    System.out.println("Hatch Claw " + state);
  }

  @Override
  protected void execute() {
    if (state == State.OPEN) {
      Robot.hatchClaw.setClawOpen();
    } else {
      Robot.hatchClaw.setClawClose();
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
