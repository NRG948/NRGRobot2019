package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDrive;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;
import jaci.pathfinder.Trajectory;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
/**
 * Tank drive subsystem.
 */
public class Drive extends Subsystem {

  public static final double DEFAULT_TURN_P = 0.081;
  public static final double DEFAULT_TURN_I = 0.00016;
  public static final double DEFAULT_TURN_D = 0.0072;
   
  public static final double DEFAULT_DRIVE_P = 0.081;
  public static final double DEFAULT_DRIVE_I = 0.00016;
  public static final double DEFAULT_DRIVE_D = 0.0072;

  public static final double DEFAULT_PATH_P = 0.4;
  public static final double DEFAULT_PATH_I = 0.00;
  public static final double DEFAULT_PATH_D = 0.00;
 
  private static final double DRIVE_WHEEL_BASE = 25.5;
  private static final double DRIVE_MAX_VELOCITY = 162;
 
  private SpeedControllerGroup leftMotor = new SpeedControllerGroup(RobotMap.driveFrontLeftMotor,RobotMap.driveMiddleLeftMotor, RobotMap.driveBackLeftMotor);
  private SpeedControllerGroup rightMotor = new SpeedControllerGroup(RobotMap.driveFrontRightMotor, RobotMap.driveMiddleRightMotor, RobotMap.driveBackRightMotor);
   private DifferentialDrive motivator = new DifferentialDrive(leftMotor, rightMotor);
  private SimplePIDController turnPIDController; 
  private SimplePIDController drivePIDController; 
  private DistanceFollower leftFollower;
  private DistanceFollower rightFollower;
  private double leftStart;
  private double rightStart;
  private double currentHeading = 0;

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
    double p = Robot.preferences.getDouble(PreferenceKeys.TURN_P_TERM, DEFAULT_TURN_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.TURN_I_TERM, DEFAULT_TURN_I);
		double d = Robot.preferences.getDouble(PreferenceKeys.TURN_D_TERM, DEFAULT_TURN_D);
    this.turnPIDController = new SimplePIDController(p, i, d).
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
    double p = Robot.preferences.getDouble(PreferenceKeys.DRIVE_P_TERM, DEFAULT_DRIVE_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.DRIVE_I_TERM, DEFAULT_DRIVE_I);
		double d = Robot.preferences.getDouble(PreferenceKeys.DRIVE_D_TERM, DEFAULT_DRIVE_D);
    this.drivePIDController = new SimplePIDController(p, i, d).
      setSetpoint(currentHeading).setAbsoluteTolerance(0);
      setCurrentHeading(currentHeading);
  }

  public void driveOnHeadingExecute(double power, double heading) {
    this.drivePIDController.setSetpoint(heading);
    setCurrentHeading(heading);
    driveOnHeadingExecute(power);
  }

  public void driveOnHeadingExecute(double power) {
    double powerDelta = this.drivePIDController.update(RobotMap.navx.getAngle());
    System.out.println(driveOnHeadingIsOnTarget());
    if(powerDelta<0){
      this.tankDrive(power+powerDelta, power);
    } else{
      this.tankDrive(power, power-powerDelta);
    }
    SmartDashboard.putNumber("Drive/driveOnHeading/PIDOutput", powerDelta);
    SmartDashboard.putNumber("Drive/driveOnHeading/PIDError", this.drivePIDController.getError());
    SmartDashboard.putNumber("Drive/driveOnHeading/Setpoint", this.drivePIDController.getSetpoint());
  }

  public boolean driveOnHeadingIsOnTarget () {
    return this.drivePIDController.onTarget();
  }

  public void driveOnHeadingEnd () {
    this.stopMotor();
    this.drivePIDController = null;
  }

  public void followTrajectoryInit(Trajectory pathName) {
    double p = Robot.preferences.getDouble(PreferenceKeys.PATH_P_TERM, DEFAULT_PATH_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.PATH_I_TERM, DEFAULT_PATH_I);
    double d = Robot.preferences.getDouble(PreferenceKeys.PATH_D_TERM, DEFAULT_PATH_D);
    
    TankModifier modifier = new TankModifier(pathName).modify(DRIVE_WHEEL_BASE);
    this.leftFollower = new DistanceFollower(modifier.getRightTrajectory()); 
    this.rightFollower = new DistanceFollower(modifier.getLeftTrajectory());
    this.leftFollower.configurePIDVA(p, i, d, 1.0 / DRIVE_MAX_VELOCITY, 0);
    this.rightFollower.configurePIDVA(p, i, d, 1.0 / DRIVE_MAX_VELOCITY, 0);
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

  private void setCurrentHeading(double heading) {
    this.currentHeading = heading;
  }

  public double getCurrentHeading() {
    return this.currentHeading;
  }
}
