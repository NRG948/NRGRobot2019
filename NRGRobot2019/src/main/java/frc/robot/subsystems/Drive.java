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
import jaci.pathfinder.Trajectory;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
/**
 * Add your docs here.
 */
public class Drive extends Subsystem {

  private final double DEFAULT_TURN_P = 0.081;
  private final double DEFAULT_TURN_I = 0.00016;
  private final double DEFAULT_TURN_D = 0.0072;
  private final double DEFAULT_DRIVE_P = 0.081;
  private final double DEFAULT_DRIVE_I = 0.00016;
  private final double DEFAULT_DRIVE_D = 0.0072;
  private final double INCHES_PER_METER = 39.37;
  private final double DEFAULT_PATH_P = 0.081;
  private final double DEFAULT_PATH_I = 0.00;
  private final double DEFAULT_PATH_D = 0.00;
  private final double DRIVE_WHEEL_BASE = 25.25;
  private final int DRIVE_TICKS_PER_REV = 1024;
  private final double DRIVE_WHEEL_DIAMETER = 8.0;
  private final double DRIVE_MAX_VELOCITY = 190;

  private SpeedControllerGroup leftMotor = new SpeedControllerGroup(RobotMap.driveFrontLeftMotor,RobotMap.driveMiddleLeftMotor, RobotMap.driveBackLeftMotor);
  private SpeedControllerGroup rightMotor = new SpeedControllerGroup(RobotMap.driveFrontRightMotor, RobotMap.driveMiddleRightMotor, RobotMap.driveBackRightMotor);
   private DifferentialDrive motivator = new DifferentialDrive(leftMotor, rightMotor);
  private SimplePIDController turnPIDController; 
  private SimplePIDController drivePIDController; 
  private DistanceFollower leftFollower;
  private DistanceFollower rightFollower;
  private double leftStart;
  private double rightStart;

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
    motivator.stopMotor();// stops the motor
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
      setSetpoint(currentHeading).setAbsoluteTolerance(0);
  }

  public void driveOnHeadingExecute(double power) {
    double powerDelta = this.drivePIDController.update(RobotMap.navx.getAngle());
    System.out.println(driveOnHeadingIsOnTarget());
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

  public void followTrajectoryInit(Trajectory pathName) {
    TankModifier modifier = new TankModifier(pathName).modify(DRIVE_WHEEL_BASE);

    this.leftFollower = new DistanceFollower(modifier.getRightTrajectory()); 
    this.rightFollower = new DistanceFollower(modifier.getLeftTrajectory());
    // this.leftFollower.configureEncoder(RobotMap.driveLeftEncoder.get(), DRIVE_TICKS_PER_REV, DRIVE_WHEEL_DIAMETER);
    // this.rightFollower.configureEncoder(RobotMap.driveRightEncoder.get(), DRIVE_TICKS_PER_REV, DRIVE_WHEEL_DIAMETER);
    this.leftFollower.configurePIDVA(DEFAULT_PATH_P, DEFAULT_PATH_I, DEFAULT_PATH_D, 1.0 / DRIVE_MAX_VELOCITY, 0);
    this.rightFollower.configurePIDVA(DEFAULT_PATH_P, DEFAULT_PATH_I, DEFAULT_PATH_D, 1.0 / DRIVE_MAX_VELOCITY, 0);
    // alignes left and right sides of the robot into the pathweaver tool.
    leftStart = RobotMap.driveLeftEncoder.getDistance();
    rightStart = RobotMap.driveRightEncoder.getDistance();
  }

  public void followTrajectoryExecute(){
    double leftPostion = leftFollower.getSegment().position;
    double rightPosition = rightFollower.getSegment().position;
    double leftEncoder = RobotMap.driveLeftEncoder.getDistance()-leftStart;
    double left = this.leftFollower.calculate(leftEncoder);
    double rightEncoder = RobotMap.driveRightEncoder.getDistance()-rightStart;
    double right = this.rightFollower.calculate(rightEncoder);
    double currentHeading = RobotMap.navx.getAngle();
    double desiredHeading = Math.toDegrees(this.leftFollower.getHeading());
    double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - currentHeading);
    double turn = 1.25 * (1.0 / 80.0) * angleDifference;
    //turn += 0.1; //attempt to compensate for drive base pulling left
   

    tankDrive(left + turn, right - turn);
    System.out.println(String.format("l: %.2f r: %.2f t: %.2f le: %.2f re: %.2f lp: %.2f rp: %.2f ch: %.1f dh: %.1f ad: %.1f"
    , left, right, turn, leftEncoder, rightEncoder, leftPostion, rightPosition, currentHeading, desiredHeading, angleDifference));
    
  }


  public boolean followTrajectoryIsFinished(){
    return this.leftFollower.isFinished() && this.rightFollower.isFinished();
  }

  public void followTrajectoryEnd(){
    stopMotor();
    this.leftFollower = null;
    this.rightFollower= null;
  }
}
