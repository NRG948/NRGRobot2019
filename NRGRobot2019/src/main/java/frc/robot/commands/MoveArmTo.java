package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.LogManager;

public class MoveArmTo extends Command {
  private static final Logger logs = LogManager.getLogger(MoveArmTo.class.getName());
 
  private Arm.Angle angle;

  public MoveArmTo(Arm.Angle angle) {
    requires(Robot.arm);
    this.angle = angle;
  }

  @Override
  protected void initialize() {
    int ticks = this.angle.getTicks();
    System.out.println("MoveArmTo " + ticks + " Init");
    Robot.arm.setSetpoint(ticks);
    logs.entry();
    logs.info("arm extended to " + this.angle + " degrees");
  }

  @Override
  protected void execute() {
    // pid controller is updated in teleopPeriodic and autonomousPeriodic
  }

  @Override
  protected boolean isFinished() {
    return Robot.arm.armPIDControllerOnTarget();
  }

  @Override
  protected void end() {
    System.out.println("MoveArmTo End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
