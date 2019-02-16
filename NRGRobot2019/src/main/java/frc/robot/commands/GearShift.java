package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Gearbox;
import frc.robot.subsystems.Gearbox.Gear;

public class GearShift extends Command {
  public Gear gear;
  public GearShift(Gear gear) {
    requires(Robot.gearbox);
    this.gear = gear;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Gear Shift" + gear);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(gear == Gear.HIGH){
      Robot.gearbox.setHighGear();
    }else{
      Robot.gearbox.setLowGear();
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
