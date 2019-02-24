package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.GearShift;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Gearbox.Gear;

public class AutonomousRoutines extends CommandGroup {
  public static final int FIELD_LENGTH_INCHES = 54 * 12;
  public static final int FIELD_WIDTH_INCHES = 27 * 12;

  private static final double TANK_POWER = 0.7;

  private AutoMovement autoMovement;
  private AutoStartingPosition autoStartingPosition;
  private AutoFeederPosition autoFeederPosition;

  /**
   * Read autonomous choosers and build a command group to perform the desired
   * autonomous routine
   */
  public AutonomousRoutines() {
    // addSequential(new SetDriveScale(Drive.SCALE_LOW));

    autoMovement = OI.getAutoMovement();
    autoStartingPosition = OI.getAutoStartingPosition();
    autoFeederPosition = OI.getAutoStationPosition();
    System.out.println("Auto Movement is : " + autoMovement);
    System.out.println("Auto Position is : " + autoStartingPosition);
    System.out.println("Auto Station position is " + autoFeederPosition);

    switch (autoMovement) {
    case NONE:
      return;

    case FORWARD:
      addSequential(new DriveStraightDistance(80, TANK_POWER), 3);
      return;

    default:
      addSequential(new GearShift(Gear.HIGH));
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoStartingPosition, autoMovement)));
      addSequential(new DeliverHatch());
      break;
    }

    switch (autoFeederPosition) {
    case NONE:
      addSequential(new GearShift(Gear.LOW));
      return;

    default:
      addSequential(new DriveStraightDistance(6, -TANK_POWER));
      addSequential(new TurnToHeading(-135, TANK_POWER));
      addSequential(new FollowPathWeaverFile(getPathWeaverFileName(autoMovement, autoFeederPosition)));
      break;
    }
    addSequential(new GearShift(Gear.LOW));
  }

  private String getPathWeaverFileName(AutoMovement from, AutoFeederPosition to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  private String getPathWeaverFileName(AutoStartingPosition from, AutoMovement to) {
    return getPathWeaverFileName(from.toString(), to.toString());
  }

  private String getPathWeaverFileName(String from, String to) {
    return "output/" + from + "_TO_" + to + ".pf1.csv";
  }
}
