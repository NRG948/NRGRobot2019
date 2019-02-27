package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.commandGroups.TestAutoPaths;
import frc.robot.Robot.AutoFeederPosition;
import frc.robot.commands.ActivateClimberPistons;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.DriveToVisionTape;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.GearShift;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.MoveArmTo;
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;
import frc.robot.commands.InterruptAllCommands;
import frc.robot.commands.ManualClimberMotor;
import frc.robot.commands.TurnToHeading;
import frc.robot.commands.DriveToVisionTape.Deliver;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Angle;
import frc.robot.subsystems.Gearbox.Gear;
import frc.robot.subsystems.HatchClawSubsystem.State;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.EXTEND;
import static frc.robot.subsystems.HatchExtensionSubsystem.State.RETRACT;

import frc.robot.utilities.MathUtil;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private Joystick leftJoystick = new Joystick(0);
  private Joystick rightJoystick = new Joystick(1);
  private XboxController xboxController = new XboxController(2);
  // assign each side of joystick to a port
  private JoystickButton driveStraightButton = new JoystickButton(leftJoystick, 1);
  private JoystickButton interruptAllCommandsButton = new JoystickButton(leftJoystick, 2); //TBD
  private JoystickButton testButton4 = new JoystickButton(leftJoystick, 7);
  private JoystickButton testButton1 = new JoystickButton(leftJoystick, 8); 
  private JoystickButton testButton2 = new JoystickButton(leftJoystick, 9); 
  private JoystickButton testButton3 = new JoystickButton(leftJoystick, 10);
  private JoystickButton resetSensorsButton = new JoystickButton(leftJoystick, 11);
  private JoystickButton testAutoPath = new JoystickButton(leftJoystick, 3);
  
  private JoystickButton gearShiftButton = new JoystickButton(rightJoystick, 1);
  private JoystickButton driveToVisionCargo = new JoystickButton(rightJoystick, 2);
  private JoystickButton driveToVisionHatch = new JoystickButton(rightJoystick, 3);
  private JoystickButton turnToHeadingButton = new JoystickButton(rightJoystick, 4);
  private JoystickButton driveStraightDistanceButton = new JoystickButton(rightJoystick, 8);
  private JoystickButton followPathButton = new JoystickButton(rightJoystick, 9);

  private JoystickButton climberPistonsExtendButton = new JoystickButton(xboxController, 8);
  private JoystickButton climberPistonsRetractButton = new JoystickButton(xboxController, 7);
  // private JoystickButton climberMotorButton = new JoystickButton(xboxController, 1); // A Button
  // private JoystickButton climberMotorButton2 = new JoystickButton(xboxController, 2); // B Button
  private JoystickButton xboxButtonA = new JoystickButton(xboxController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(xboxController, 2); // B Button
  private JoystickButton hatchOpenButton = new JoystickButton(xboxController, 3); // TBD joystick button numbers, the X buttong.
  private JoystickButton hatchCloseButton = new JoystickButton(xboxController, 4); // The Y button.
  private JoystickButton hatchExtensionButton = new JoystickButton(xboxController, 6);



  OI() {
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      RobotMap.resetSensors();
    }));
    xboxButtonA.whenPressed(new InstantCommand(() -> {
      Angle angle = this.getXboxLeftBumper() ? Arm.Angle.ARM_ROCKET_CARGO_LOW_ANGLE : Arm.Angle.ARM_STOWED_ANGLE;
      new MoveArmTo(angle).start();
    }));

    driveStraightButton.whenActive(new DriveStraight());
    driveStraightButton.whenInactive(new ManualDrive());
    turnToHeadingButton.whenPressed(new TurnToHeading(90, 1.0));
    driveStraightDistanceButton.whenPressed(new DriveStraightDistance(240, 0.7));
    hatchOpenButton.whenPressed(new HatchClaw(State.OPEN));
    hatchCloseButton.whenPressed(new HatchClaw(State.CLOSE));
    hatchExtensionButton.whenPressed(new HatchExtension(EXTEND));
    hatchExtensionButton.whenReleased(new HatchExtension(RETRACT));
    gearShiftButton.whenPressed(new GearShift(Gear.HIGH));
    gearShiftButton.whenReleased(new GearShift(Gear.LOW));

    driveToVisionCargo.whenPressed(new DriveToVisionTape(Deliver.Cargo));
    driveToVisionHatch.whenPressed(new DriveToVisionTape(Deliver.Hatch));

    testAutoPath.whenPressed(new TestAutoPaths());

    interruptAllCommandsButton.whenPressed(new InterruptAllCommands());
    
    climberPistonsExtendButton.whenPressed(new ActivateClimberPistons(true));
    climberPistonsRetractButton.whenPressed(new ActivateClimberPistons(false));

    // climberMotorButton.whileHeld(new ManualClimberMotor(0.25)); //TBD
    // climberMotorButton2.whileHeld(new ManualClimberMotor(-0.25)); //TBD

    testButton1.whenPressed(new MoveArmTo(Arm.Angle.ARM_ACQUIRE_CARGO_ANGLE));
    testButton2.whenPressed(new MoveArmTo(Arm.Angle.ARM_FORWARD_ANGLE));
    testButton3.whenPressed(new MoveArmTo(Arm.Angle.ARM_STOWED_ANGLE));
    testButton4.whenPressed(new MoveArmTo(Arm.Angle.ARM_ROCKET_CARGO_MEDIUM_ANGLE));
  }

  /** Gets the Y value of the left joystick. */
  public double getLeftJoystickY() {
    return -leftJoystick.getY();
  }

  // gets the Y value of the right joystick
  public double getRightJoystickY() {
    return -rightJoystick.getY();
  }
  
  public double getXboxLeftY() {
    return -xboxController.getY(Hand.kLeft);
  }

  public double getXboxRightY() {
    return -xboxController.getY(Hand.kRight);
  }

  public double getXboxLeftTrigger() {
    return MathUtil.deadband(xboxController.getRawAxis(3), 0.05);//axis was 2 but changed to 3
  }

  public double getXboxRightTrigger() {
    return MathUtil.deadband(xboxController.getRawAxis(2), 0.05);
  }
  public boolean getXboxLeftBumper(){
    return xboxController.getBumper(Hand.kLeft);
  }

  public static AutoStartingPosition getAutoStartingPosition() {
		return Robot.autoStartingPositionChooser.getSelected();
	}

	public static AutoMovement getAutoMovement() {
		return Robot.autoMovementChooser.getSelected();
  }

  public static AutoMovement getAutoMovement2() {
    return Robot.autoMovement2Chooser.getSelected();
  }
  
  public static AutoFeederPosition getAutoStationPosition(){
    return Robot.autoStationPositionChooser.getSelected();
  }
}
