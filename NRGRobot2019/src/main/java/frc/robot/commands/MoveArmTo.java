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
    System.out.println("MoveArm " + ticks + " Init");
    Robot.arm.armAnglePIDInit(ticks, Arm.DEFAULT_ARM_TICK_TOLORANCE);
  }

  @Override
  protected void execute() {
    Robot.arm.armAnglePIDExecute();
  }

  @Override
  protected boolean isFinished() {
    return Robot.arm.armPIDControllerOnTarget();
  }

  @Override
  protected void end() {
    Robot.arm.armAnglePIDEnd();
    System.out.println("MoveArm End");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
