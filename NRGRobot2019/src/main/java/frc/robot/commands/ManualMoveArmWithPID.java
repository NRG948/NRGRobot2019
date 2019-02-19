package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

/**
 * Command that moves the arm with PID when manually controlled with xbox triggers.
 */

public class ManualMoveArmWithPID extends Command {
  /*calculation: 2600 ticks is full range of the arm
  * we want 1300 ticks per second, to move the whole arm in two seconds
  * that's 1300 ticks/second * 1 second/1000 milliseconds * 20 milliseconds = 26 ticks
  */
  private static final int MAX_TICKS_PER_CYCLE = 26;
  private double lastArmSpeed;

  public ManualMoveArmWithPID() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize() {
    System.out.println("ManualMoveArmWithPID init");
  }

  @Override
  protected void execute() {
    double upSpeed = Robot.oi.getXboxRightTrigger();
    double downSpeed = Robot.oi.getXboxLeftTrigger();
    SmartDashboard.putNumber("Arm Angle PID/RightTrigger", upSpeed);
    SmartDashboard.putNumber("Arm Angle PID/LeftTrigger", downSpeed);
    double speed = upSpeed - downSpeed;
    if(speed < 0) {
      Robot.arm.setPIDOutputLimits(speed);
      Robot.arm.setSetpoint(Arm.Angle.ARM_MAX_ANGLE.getTicks());
    } else if(speed > 0) {
      Robot.arm.setPIDOutputLimits(speed);
      Robot.arm.setSetpoint(Arm.Angle.ARM_STOWED_ANGLE.getTicks());
    } else if(speed != this.lastArmSpeed){
      Robot.arm.setPIDOutputLimits(1.0);
      Robot.arm.setSetpoint(Robot.arm.getCurrentArmPosition());
    }
    this.lastArmSpeed = speed;
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    System.out.println("ManualMoveArmWithPID end");
  }

  @Override
  protected void interrupted() {
    end();
  }
}
