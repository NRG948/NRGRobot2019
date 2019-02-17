package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new ManualClimb(0));
  }

  public void rawClimb(double power){
    RobotMap.climberMotor.set(power);
  }
  
  public void stop(){
    RobotMap.climberMotor.stopMotor();
  }
}
