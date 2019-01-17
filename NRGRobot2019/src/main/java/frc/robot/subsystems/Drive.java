/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrive;
import frc.robot.utilities.SimplePIDController;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  private final double DEFAULT_TURN_P = 0.2;
  private final double DEFAULT_TURN_I = 0;
  private final double DEFAULT_TURN_D = 0;
  private final double DEFAULT_DRIVE_P = 0.2;
  private final double DEFAULT_DRIVE_I = 0;
  private final double DEFAULT_DRIVE_D = 0;

  private SpeedControllerGroup leftMotor = new SpeedControllerGroup(RobotMap.driveFrontLeftMotor,RobotMap.driveBackLeftMotor);
  private SpeedControllerGroup rightMotor = new SpeedControllerGroup(RobotMap.driveFrontRightMotor, RobotMap.driveBackRightMotor);
  private DifferentialDrive motivator = new DifferentialDrive(leftMotor, rightMotor);
  private SimplePIDController turnPIDController; 
  private SimplePIDController drivePIDController; 

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualDrive());
  }

  public void tankDrive (double leftPower, double rightPower) {
    motivator.tankDrive(leftPower, rightPower);

  }
  public void stopMotor () {
    motivator.stopMotor();
  }

  public void turnToHeadingInit(double desiredHeading, double tolerance) {
    this.turnPIDController = new SimplePIDController(DEFAULT_TURN_P, DEFAULT_TURN_I, DEFAULT_TURN_D).
      setSetpoint(desiredHeading).
      setAbsoluteTolerance(tolerance);
  }

  public void turnToHeadingExecute(double maxPower){
    double currentPower = this.turnPIDController.update(RobotMap.navx.getAngle()) * maxPower;
    this.tankDrive(currentPower, -currentPower); 
  }

  public boolean turnToHeadingOnTarget() {
    return this.turnPIDController.onTarget();
  }

  public void turnToHeadingEnd() {
    this.stopMotor();
    this.turnPIDController = null;
  }

  public void driveOnHeadingInit(double currentHeading){
    this.drivePIDController = new SimplePIDController(DEFAULT_DRIVE_P, DEFAULT_DRIVE_I, DEFAULT_DRIVE_D).
      setSetpoint(currentHeading);
  }

  public void driveOnHeadingExecute(double power) {
    double powerDelta = this.drivePIDController.update(RobotMap.navx.getAngle());
    if(powerDelta<0){
      this.tankDrive(power+powerDelta, power);
    } else{
      this.tankDrive(power, power-powerDelta);
    }
  }

  public boolean driveOnHeadingIsOnTarget () {
    return this.drivePIDController.onTarget();
  }

  public void driveOnHeadingEnd () {
    this.stopMotor();
    this.drivePIDController = null;
  }
}
