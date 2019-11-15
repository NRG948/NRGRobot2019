package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.HatchExtensionSubsystem.State;


//import statements for logger
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;
 

/**
 * Command to extend and retract the hatch extender.
 */

public class HatchExtension extends Command {
  
  static final Logger logger = LogManager.getLogger(HatchExtension.class.getName());

  private State state;

  public HatchExtension(State state) {
    requires(Robot.hatchExtension);
    this.state = state;
  }

  @Override
  protected void initialize() {
    System.out.println("Hatch Extension  " + state);
  }

  @Override
  protected void execute() {
    if (state == State.EXTEND) {
      Robot.hatchExtension.extend();
      logger.entry();
      logger.info("hatch extended");
    } else {
      Robot.hatchExtension.retract();
      logger.entry();
      logger.info("retracted");
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
