package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class MoveArmTo extends Command {
  private int ticks;

  public MoveArmTo(Arm.Angle angle) {
    this(angle.getTicks());
  }
  
  public MoveArmTo(int ticks) {
    requires(Robot.arm);
    this.ticks = ticks;
  }

  @Override
  protected void initialize() {
    System.out.println("MoveArmTo " + ticks + " Init");
    Robot.arm.setSetpoint(this.ticks);
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
