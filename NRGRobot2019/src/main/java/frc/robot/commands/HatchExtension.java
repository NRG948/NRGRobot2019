package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.HatchExtensionSubsystem;
import frc.robot.subsystems.HatchExtensionSubsystem.State;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class HatchExtension extends Command {
  private State state;
  public HatchExtension(State state) {
   requires(Robot.hatchExtension);
   this.state = state;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Hatch Extension Init" + state);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(state == State.EXTENDED) {
      Robot.hatchExtension.extend();
    } else {
      Robot.hatchExtension.retract();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
