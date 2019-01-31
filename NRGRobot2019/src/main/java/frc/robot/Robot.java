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
import frc.robot.subsystems.Drive;
import frc.robot.utilities.PositionTracker;
import frc.robot.vision.I2Cwrapper;
import frc.robot.vision.ColorSensor.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drive drive;
  public static OI oi;
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
    SmartDashboard.putData("LeftEncoder", RobotMap.driveLeftEncoder);
    SmartDashboard.putData("RightEncoder", RobotMap.driveRightEncoder);
    SmartDashboard.putNumber("Gyro", RobotMap.navx.getAngle());
    SmartDashboard.putData("DriveSubsystem", Robot.drive);

    Color color = RobotMap.colorSensor.read();
    SmartDashboard.putNumber("ColorSensor/Red", color.red);
    SmartDashboard.putNumber("ColorSensor/Green",color.green);
    SmartDashboard.putNumber("ColorSensor/Blue", color.blue);
    SmartDashboard.putNumber("ColorSensor/Prox", color.prox);

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
    positionTracker.updatePosition();
    Scheduler.getInstance().run();

    // RobotMap.colorSensor.updateColorSensor();
    // RobotMap.colorSensor2.updateColorSensor();
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
    positionTracker.updatePosition();
    Scheduler.getInstance().run();

    // RobotMap.colorSensor.updateColorSensor();
    // RobotMap.colorSensor2.updateColorSensor();
  }

  @Override
  public void testPeriodic() {
    positionTracker.updatePosition();

    // RobotMap.colorSensor.updateColorSensor();
    // RobotMap.colorSensor2.updateColorSensor();
  }
}
