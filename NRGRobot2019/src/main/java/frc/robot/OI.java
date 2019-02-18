package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot.AutoMovement;
import frc.robot.Robot.AutoStartingPosition;
import frc.robot.commands.ActivateClimberPistons;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.HatchClaw;
import frc.robot.commands.HatchExtension;
import frc.robot.commands.ManualClimberMotor;
import frc.robot.commands.TurnToHeading;
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
  private JoystickButton resetSensorsButton = new JoystickButton(leftJoystick, 11);
  private JoystickButton interruptAllCommandsButton = new JoystickButton(leftJoystick, 2); //TBD
  private JoystickButton testButton1 = new JoystickButton(leftJoystick, 8); 
  private JoystickButton testButton2 = new JoystickButton(leftJoystick, 9); 
  private JoystickButton testButton3 = new JoystickButton(leftJoystick, 10); 

  private JoystickButton driveStraightButton = new JoystickButton(rightJoystick, 1);
  private JoystickButton turnToHeadingButton = new JoystickButton(rightJoystick, 3);
  private JoystickButton driveStraightDistanceButton = new JoystickButton(rightJoystick, 8);
  private JoystickButton followPathButton = new JoystickButton(rightJoystick, 9);

  private JoystickButton climberPistonsExtendButton = new JoystickButton(xboxController, 8);
  private JoystickButton climberPistonsRetractButton = new JoystickButton(xboxController, 7);
  private JoystickButton climberMotorButton = new JoystickButton(xboxController, 1);
  private JoystickButton hatchOpenButton = new JoystickButton(xboxController, 3); // TBD joystick button numbers, the X buttong.
  private JoystickButton hatchCloseButton = new JoystickButton(xboxController, 4); // The Y button.
  private JoystickButton hatchExtensionButton = new JoystickButton(xboxController, 9);

  OI() {
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      RobotMap.resetSensors();
    }));

    driveStraightButton.whenActive(new DriveStraight());
    driveStraightButton.whenInactive(new ManualDrive());
    turnToHeadingButton.whenPressed(new TurnToHeading(90, 1.0));
    driveStraightDistanceButton.whenPressed(new DriveStraightDistance(120, 0.7));
    followPathButton.whenPressed(new FollowPathWeaverFile("output/CrazyAuto.pf1.csv"));
    hatchOpenButton.whenPressed(new HatchClaw(State.OPEN));
    hatchCloseButton.whenPressed(new HatchClaw(State.CLOSE));
    hatchExtensionButton.whenPressed(new HatchExtension(EXTEND));
    hatchExtensionButton.whenReleased(new HatchExtension(RETRACT));
    
    climberPistonsExtendButton.whenPressed(new ActivateClimberPistons(true));
    climberPistonsRetractButton.whenPressed(new ActivateClimberPistons(false));

    climberMotorButton.whileHeld(new ManualClimberMotor(0.25)); //TBD

    testButton1.whenPressed(new MoveArm(100));
    testButton2.whenPressed(new MoveArm(200));
    testButton3.whenPressed(new MoveArm(300));
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
    return MathUtil.deadband(xboxController.getRawAxis(2), 0.05);
  }

  public double getXboxRightTrigger() {
    return MathUtil.deadband(xboxController.getRawAxis(3), 0.05);
  }

  public static AutoStartingPosition getAutoStartingPosition() {
		return Robot.autoStartingPositionChooser.getSelected();
	}

	public static AutoMovement getAutoMovement() {
		return Robot.autoMovementChooser.getSelected();
	}
}
