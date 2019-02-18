package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ManualClimberMotor;

/**
 * Subsystem that controls climber motor.
 */
public class ClimberMotor extends Subsystem {

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualClimberMotor(0));
  }

  public void rawClimb(double power) {
    RobotMap.climberMotor.set(power);
  }

  public void stop() {
    RobotMap.climberMotor.stopMotor();
  }
}