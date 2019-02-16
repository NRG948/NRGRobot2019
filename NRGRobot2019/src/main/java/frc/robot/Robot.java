/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Sendable;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CargoAcquirer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gearbox;
import frc.robot.subsystems.HatchClawSubsystem;
import frc.robot.subsystems.HatchExtensionSubsystem;
import frc.robot.utilities.PositionTracker;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;

  public static Gearbox gearbox;
  public static Drive drive;
  public static CargoAcquirer cargoAcquirer;
  public static Arm arm;
  public static HatchClawSubsystem hatchClaw;
  public static HatchExtensionSubsystem hatchExtension;

  public static PositionTracker positionTracker = new PositionTracker();
  public static PowerDistributionPanel pdp = new PowerDistributionPanel();

  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.init();
    drive = new Drive();
    gearbox = new Gearbox();
    arm = new Arm();
    cargoAcquirer = new CargoAcquirer(); 
    hatchClaw = new HatchClawSubsystem();
    hatchExtension = new HatchExtensionSubsystem();
    

    chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", chooser);
    System.out.println("robotInit()");
    oi = new OI();
    LiveWindow.addSensor("pdp", "pdp", Robot.pdp);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("PositionTracker/x", positionTracker.getX());
    SmartDashboard.putNumber("PositionTracker/y", positionTracker.getY());
    SmartDashboard.putNumber("PositionTracker/maxVelocity", positionTracker.getMaxVelocity());
    SmartDashboard.putData("LeftEncoder", RobotMap.driveLeftEncoder);
    SmartDashboard.putData("RightEncoder", RobotMap.driveRightEncoder);
    SmartDashboard.putNumber("Gyro", RobotMap.navx.getAngle());
    SmartDashboard.putData("DriveSubsystem", Robot.drive);
    positionTracker.updatePosition();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    System.out.println("autonomousInit()");
    autonomousCommand = chooser.getSelected();

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    System.out.println("teleopInit()");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
    positionTracker.updatePosition();
  }

  public void initPreferences() {
		if (preferences.getBoolean(PreferenceKeys.WRITE_DEFAULT, true)) {
			preferences.putDouble(PreferenceKeys.ARM_P_TERM, Arm.DEFAULT_ARM_P);
			preferences.putDouble(PreferenceKeys.ARM_I_TERM, Arm.DEFAULT_ARM_I);
			preferences.putDouble(PreferenceKeys.ARM_D_TERM, Arm.DEFAULT_ARM_D);
			preferences.putDouble(PreferenceKeys.ARM_UP_MAX_POWER, Arm.LIFT_POWER_SCALE_UP);
			preferences.putDouble(PreferenceKeys.ARM_DOWN_MAX_POWER, Arm.LIFT_POWER_SCALE_DOWN);

			preferences.putDouble(PreferenceKeys.TURN_P_TERM, Drive.DEFAULT_TURN_P);
			preferences.putDouble(PreferenceKeys.TURN_I_TERM, Drive.DEFAULT_TURN_I);
			preferences.putDouble(PreferenceKeys.TURN_D_TERM, Drive.DEFAULT_TURN_D);
			preferences.putDouble(PreferenceKeys.DRIVE_TURN_MAX_POWER, Drive.DEFAULT_DRIVE_TURN_POWER);

			preferences.putDouble(PreferenceKeys.DRIVE_P, Drive.DEFAULT_DRIVE_P);
			preferences.putDouble(PreferenceKeys.DRIVE_I, Drive.DEFAULT_DRIVE_I);
			preferences.putDouble(PreferenceKeys.DRIVE_D, Drive.DEFAULT_DRIVE_D);
			preferences.putDouble(PreferenceKeys.DRIVE_MAX_POWER, Drive.DEFAULT_DRIVE_POWER);

			preferences.putDouble(PreferenceKeys.DRIVE_Y_P, Drive.DEFAULT_DRIVE_Y_P);
			preferences.putDouble(PreferenceKeys.DRIVE_Y_I, Drive.DEFAULT_DRIVE_Y_I);
			preferences.putDouble(PreferenceKeys.DRIVE_Y_D, Drive.DEFAULT_DRIVE_Y_D);
			preferences.putDouble(PreferenceKeys.DRIVE_Y_MAX_POWER, Drive.DEFAULT_DRIVE_Y_POWER);

			preferences.putDouble(PreferenceKeys.DRIVEYH_X, 48.0);
			preferences.putDouble(PreferenceKeys.DRIVEYH_Y, 48.0);
			preferences.putDouble(PreferenceKeys.DRIVEYH_H, 0);
			preferences.putDouble(PreferenceKeys.DRIVEYH_X_POWER, 0.9);
			preferences.putDouble(PreferenceKeys.DRIVEYH_Y_POWER, 0.5);
			preferences.putDouble(PreferenceKeys.DRIVEYH_TURN_POWER, 0.3);

			preferences.putInt(PreferenceKeys.SWITCH_TICKS, CubeLifter.DEFAULT_SWITCH_TICKS);
			preferences.putInt(PreferenceKeys.SCALE_HIGH_TICKS, CubeLifter.DEFAULT_SCALE_HIGH_TICKS);
			preferences.putInt(PreferenceKeys.SCALE_MEDIUM_TICKS, CubeLifter.DEFAULT_SCALE_MEDIUM_TICKS);
			preferences.putInt(PreferenceKeys.SCALE_LOW_TICKS, CubeLifter.DEFAULT_SCALE_LOW_TICKS);
			preferences.putInt(PreferenceKeys.STOWED_TICKS, CubeLifter.DEFAULT_STOWED_TICKS);

			preferences.putBoolean(PreferenceKeys.USE_PHYSICAL_AUTO_CHOOSER, true);
			preferences.putBoolean(PreferenceKeys.USE_FOUR_ENCODERS, false);
			preferences.putBoolean(PreferenceKeys.USING_PRACTICE_BOT, true);
			
			preferences.putBoolean(PreferenceKeys.WRITE_DEFAULT, false);
			
			preferences.putDouble(PreferenceKeys.AUTO_MAX_DRIVE_ACCEL, Drive.DEF_AUTO_MAX_DRIVE_ACCEL);
			preferences.putDouble(PreferenceKeys.TELEOP_DRIVE_ACCEL_MAX_ARM_HEIGHT, Drive.DEF_TELEOP_DRIVE_ACCEL_MAX_ARM_HEIGHT);

			preferences.putDouble(PreferenceKeys.MEC_ENCODER_LF_RATIO_PRACTICE, RobotMap.DEF_MEC_ENCODER_LF_RATIO_PRACTICE);
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_LR_RATIO_PRACTICE, RobotMap.DEF_MEC_ENCODER_LR_RATIO_PRACTICE);
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_RF_RATIO_PRACTICE, RobotMap.DEF_MEC_ENCODER_RF_RATIO_PRACTICE);
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_RR_RATIO_PRACTICE, RobotMap.DEF_MEC_ENCODER_RR_RATIO_PRACTICE);
			
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_LF_RATIO_COMP, RobotMap.DEF_MEC_ENCODER_LF_RATIO_COMP);
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_LR_RATIO_COMP, RobotMap.DEF_MEC_ENCODER_LR_RATIO_COMP);
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_RF_RATIO_COMP, RobotMap.DEF_MEC_ENCODER_RF_RATIO_COMP);
			preferences.putDouble(PreferenceKeys.MEC_ENCODER_RR_RATIO_COMP, RobotMap.DEF_MEC_ENCODER_RR_RATIO_COMP);
			
			preferences.putInt(PreferenceKeys.PIXY_CAM_CUBE_SIGNATURE, 1);
		}
}
