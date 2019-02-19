package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class MoveArmTo extends Command {
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
