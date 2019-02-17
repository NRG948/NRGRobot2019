/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.FollowPathWeaverFile;

public class AutonomousRoutines extends CommandGroup {
  public static final int FIELD_LENGTH_INCHES = 54 * 12;
	public static final int FIELD_WIDTH_INCHES = 27 * 12;

  private static final double TANK_POWER = 0.7;
  

  private AutoMovement autoMovement;
  private AutoStartingPosition autoStartingPosition;
  
  /**
   * Read autonomous choosers and build a command group to perform the desired autonomous routine
   */
  public AutonomousRoutines() {
    // addSequential(new SetDriveScale(Drive.SCALE_LOW));

    autoMovement = OI.getAutoMovement();
    autoStartingPosition = OI.getAutoStartingPosition();
    System.out.println("Auto Movement is : " + autoMovement);
    System.out.println("Auto Position is : " + autoStartingPosition);

    switch (autoMovement) {
      case NONE:
        return;

      case FORWARD:
        addSequential(new DriveStraightDistance(80, TANK_POWER), 3);
        return;

      case CARGO_FRONT_LEFT_HATCH:
        addSequential(new FollowPathWeaverFile("output/Cargo_Front_Left_From_" + autoStartingPosition + ".pf1.csv"));
        // addSequential(new DriveToVisionTarget());
        addSequential(new DeliverHatch());
        return;

      case CARGO_FRONT_RIGHT_HATCH:
        addSequential(new FollowPathWeaverFile("output/Cargo_Front_Right_From_" + autoStartingPosition + ".pf1.csv"));
        // addSequential(new DriveToVisionTarget());
        addSequential(new DeliverHatch());
        return;
    }
  }
}
