package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Gearbox.Gear;

public class GearShift extends Command {
  public Gear gear;
  public GearShift(Gear gear) {
    requires(Robot.gearbox);
    this.gear = gear;
  }

  @Override
  protected void initialize() {
    System.out.println("Gear Shift " + gear);
  }

  @Override
  protected void execute() {
    if(gear == Gear.HIGH){
      Robot.gearbox.setHighGear();
    }else{
      Robot.gearbox.setLowGear();
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
  }
}
