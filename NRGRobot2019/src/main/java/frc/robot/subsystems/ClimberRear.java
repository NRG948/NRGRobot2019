package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualClimbRear;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Subsystem that controls climber motor.
 */

public class ClimberRear extends Subsystem {

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualClimbRear(0));
  }
// negative means drive the rack and pinion downward, lifting the robot
  public void rawClimb(double power) {
    RobotMap.climberRearMotor.set(power);
  }

  public void stop() {
    RobotMap.climberRearMotor.stopMotor();
  }
}