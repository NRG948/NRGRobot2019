// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.command.Subsystem;
// import frc.robot.commands.ManualCargoAcquirer;

// /**
//  * Add your docs here.
//  */
// public class CargoAcquirer extends Subsystem {
//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.
//   public enum Direction {
//     ACQUIER, EJECT;
//   }

//   @Override
//   public void initDefaultCommand() {
//     // Set the default command for a subsystem here.
//     setDefaultCommand(new ManualCargoAcquirer());
//   }
//   public void acquier(double power, Direction direction){
//     power = Math.abs(power);

//     if (direction == Direction.ACQUIER) {
//       rawAcquire(-power, -power);
//     } else {
//       rawAcquire(power,power);
//   }
// }
  
//   private void rawAcquire(double leftPower, double rightPower) {
//   }

//   public void stop() {
    
//   }
// }
