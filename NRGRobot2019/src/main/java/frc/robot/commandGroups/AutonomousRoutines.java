package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.Robot.DelayBeforeAuto;
import frc.robot.Robot.HabitatLevel;
import frc.robot.commands.DelaySeconds;
import frc.robot.commands.DriveDistanceOnHeading;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.GearShift;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.Gearbox.Gear;

public class AutonomousRoutines extends CommandGroup {
  public static final int FIELD_LENGTH_INCHES = 54 * 12;
  public static final int FIELD_WIDTH_INCHES = 27 * 12;

  private static final double DRIVE_POWER = 0.8;
  private static final double TURN_POWER = 1.0;

  /**
   * Read autonomous choosers and build a command group to perform the desired
   * autonomous routine
   */
  public AutonomousRoutines() {
    double drivePower = DRIVE_POWER;

    double turnPower = TURN_POWER;
    if (!Robot.drive.areTurnInputsSquared()) {
      turnPower *= turnPower;
    }

    AutoMovement autoMovement = OI.getAutoMovement();
    AutoStartingPosition autoStartingPosition = OI.getAutoStartingPosition();
    AutoFeederPosition autoFeederPosition = OI.getAutoStationPosition();
    AutoMovement autoMovement2 = OI.getAutoMovement2();
    HabitatLevel habLevel = OI.getAutoHabitatLevel();
    DelayBeforeAuto delayBeforeAuto = OI.getDelayBeforeAuto();

    System.out.println("Auto Movement is : " + autoMovement);
    System.out.println("Auto Position is : " + autoStartingPosition);
    System.out.println("Auto Station position is " + autoFeederPosition);
    System.out.println("Auto Movement 2 is : " + autoMovement2);
    System.out.println("Auto Habitat level is: " + habLevel);

    addSequential(new GearShift(Gear.HIGH));
    if (habLevel != HabitatLevel.LEVEL_1 && autoMovement != AutoMovement.NONE) {
      addSequential(new DriveStraightDistance(40, 0.7, false));
    }

    // Handle delay for auto
    switch (delayBeforeAuto) {
    case ZERO:
      return;

    case FIVE:
      addSequential(new DelaySeconds(5.0));

    case TEN:
      addSequential(new DelaySeconds(10.0));
    }
    
    // Handle movement from starting position
    switch (autoMovement) {
    case NONE:
      return;

    case FORWARD:
      addSequential(new DriveStraightDistance(80, drivePower), 3);
      return;

    case ROCKET_CLOSE:
      addSequential(
          new DriveDistanceOnHeading(autoStartingPosition == AutoStartingPosition.RIGHT ? 40.0 : -40.0, 120, 1.0, 2.0));
      addSequential(new DeliverHatch());
      break;

    case CARGO_FIRST_HATCH_CLOSE:
      addSequential(new DriveDistanceOnHeading(autoStartingPosition == AutoStartingPosition.RIGHT ? 10.0 : -10.0, 193.0,
          drivePower, 2.0));
      addSequential(new TurnToHeading(autoStartingPosition == AutoStartingPosition.RIGHT ? -90.0 : 90.0, turnPower));
      addSequential(new DeliverHatch());
      return;

    default:
      switch (autoStartingPosition) {
      case LEFT:
        addSequential(new DriveStraightDistance(10.0, 18, drivePower, false));
        addSequential(new DriveStraightDistance(30.0, 58.0, drivePower * 1.2, false));
        addSequential(new DriveDistanceOnHeading(0.0, 32.0, drivePower * 1.2, 1.5));
        break;

      case CENTER:
        addSequential(new DriveDistanceOnHeading(autoMovement == AutoMovement.CARGO_FRONT_RIGHT_HATCH ? 6 : -6, 90,
            drivePower, 1.5));
        break;

      case RIGHT:
        addSequential(new DriveStraightDistance(-10.0, 18, drivePower, false));
        addSequential(new DriveStraightDistance(-30.0, 58.0, drivePower * 1.2, false));
        addSequential(new DriveDistanceOnHeading(0.0, 32.0, drivePower * 1.2, 1.5));
        break;
      }
      addSequential(new DeliverHatch());
      break;
    }

    // Handle feeder station movement
    switch (autoFeederPosition) {
    case NONE:
      return;

    case RIGHT_FEEDER:
      if (autoMovement == AutoMovement.CARGO_FRONT_RIGHT_HATCH) {
        addSequential(new DriveStraightDistance(-60.0, 120.0, -1.0, false));
        addSequential(new TurnToHeading(-180, turnPower));
        addSequential(new DriveDistanceOnHeading(-180.0, 44.0, drivePower * 0.9, 1.5));
        addSequential(new PickupHatch());
        // addSequential(new DriveStraightDistance(6, -drivePower));
        break;
      } else if (autoMovement == AutoMovement.ROCKET_CLOSE) {
        addSequential(new TurnToHeading(180, turnPower));
        addSequential(new DriveDistanceOnHeading(-180, 108, 1.0, 3.0));
        break;
      }
      return;

    case LEFT_FEEDER:
      if (autoMovement == AutoMovement.CARGO_FRONT_LEFT_HATCH) {
        addSequential(new DriveStraightDistance(60.0, 120.0, -1.0, false));
        addSequential(new TurnToHeading(180, turnPower));
        addSequential(new DriveDistanceOnHeading(180.0, 44.0, drivePower * 0.9, 1.5));
        addSequential(new PickupHatch());
        // addSequential(new DriveStraightDistance(6, -drivePower));
        break;
      } else if (autoMovement == AutoMovement.ROCKET_CLOSE) {
        addSequential(new TurnToHeading(-180, turnPower));
        addSequential(new DriveDistanceOnHeading(180, 108, 1.0, 3.0));
        break;
      }
      return;
    }

    // Handle movement from feeder station
    switch (autoMovement2) {
    case NONE:
    default:
      return;

    case CARGO_FIRST_HATCH_CLOSE:
      addSequential(new FeederStationToCargoFirstHatchClose(autoFeederPosition, -1, turnPower));
      break;
    }
  }
}
