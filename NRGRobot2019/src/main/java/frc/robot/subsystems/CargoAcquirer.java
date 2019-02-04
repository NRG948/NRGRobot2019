// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ManualCargoAcquirer;

/**
 * This is Subsystem for aquiring cargo.
 */
public class CargoAcquirer extends Subsystem {

  public enum Direction {
    ACQUIRE, EJECT;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualCargoAcquirer());
  }

  public void acquire(double power, Direction direction){
    power = Math.abs(power);

    if (direction == Direction.ACQUIRE) {
      rawAcquire(-power);
    } else {
      rawAcquire(power);
    }
  }
  
  public void rawAcquire(double power) {
    //RobotMap.acquireMotor.set(power);
  }

  public void stop() {
    //RobotMap.acquireMotor.set(0);
  }
}
