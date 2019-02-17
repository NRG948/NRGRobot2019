package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualMoveArm;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

/**
 * Subsystem, moves the arm.
 */
public class Arm extends Subsystem {
	public static final double ARM_UP_MAX_POWER = 0.35;
	public static final double ARM_DOWN_MAX_POWER = 0.35;
	public static final double DEFAULT_ARM_P = 0.005;
	public static final double DEFAULT_ARM_I = DEFAULT_ARM_P / 10;
	public static final double DEFAULT_ARM_D = 0;

	public static final int DEFAULT_ARM_STOWED_TICKS = 0;
	public static final int DEFAULT_ARM_CARGO_SHIP_TICKS = 100;
	public static final int DEFAULT_ARM_ROCKET_CARGO_LOW_TICKS = 200;
	public static final int DEFAULT_ARM_ROCKET_CARGO_MEDIUM_TICKS = 300;
	public static final int DEFAULT_ARM_ROCKET_CARGO_HIGH_TICKS = 400;
	public static final int DEFAULT_ARM_TICK_TOLORANCE = 5; // TODO : figure out a good value line 21-26

  private SimplePIDController pidController;

	public enum Angle {
		
		ARM_STOWED_TICKS(PreferenceKeys.ARM_STOWED_TICKS, DEFAULT_ARM_STOWED_TICKS),
		ARM_CARGO_SHIP_TICKS(PreferenceKeys.ARM_CARGO_SHIP_TICKS, DEFAULT_ARM_CARGO_SHIP_TICKS),
		ARM_ROCKET_CARGO_LOW_TICKS(PreferenceKeys.ARM_ROCKET_CARGO_LOW_TICKS, DEFAULT_ARM_ROCKET_CARGO_LOW_TICKS),
		ARM_ROCKET_CARGO_MEDIUM_TICKS(PreferenceKeys.ARM_ROCKET_CARGO_MEDIUM_TICKS, DEFAULT_ARM_ROCKET_CARGO_MEDIUM_TICKS),
		ARM_ROCKET_CARGO_HIGH_TICKS(PreferenceKeys.ARM_ROCKET_CARGO_HIGH_TICKS, DEFAULT_ARM_ROCKET_CARGO_HIGH_TICKS);
		
		public final String preferenceKey;
		public final int defaultTicks;

		private Angle(String prefKey, int defaultTicks) {
			this.preferenceKey = prefKey;
			this.defaultTicks = defaultTicks;
		}
		
		public int getTicks() {
			return Robot.preferences.getInt(preferenceKey, defaultTicks);
		}
	}

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualMoveArm());
  }

  // Positive power means up from the starting (stowed) position.
  public void moveArm(double power){
      rawMoveArm(power);
  }
  
  private void rawMoveArm(double power) {
	  if (power < 0) {
		  if (atBackLimit()) {
			  power = 0;
		  }
	  }
	  else {
		  if (atFrontLimit()) {
			  power = 0;
		  }
	  }
	  if (power != 0){
		RobotMap.armMotor.set(power);
	  }
	  else {
		  stop();
	  }
  }

  public void stop() {
  	RobotMap.armMotor.stopMotor();
  }

  public void armPIDControllerInit(double p, double i, double d, double setpoint, double tolerance) {
    double maxPowerUp = Robot.preferences.getDouble(PreferenceKeys.ARM_UP_MAX_POWER, ARM_UP_MAX_POWER);
		double maxPowerDown = Robot.preferences.getDouble(PreferenceKeys.ARM_DOWN_MAX_POWER, ARM_DOWN_MAX_POWER);
		
		pidController = new SimplePIDController(p, i, d, true)
								.setOutputRange(-maxPowerDown, maxPowerUp)
								.setAbsoluteTolerance(tolerance)
								.setSetpoint(setpoint)
								.start();
  } 

  public void armAnglePIDInit(double setpoint, double tolerance) {
		double p = Robot.preferences.getDouble(PreferenceKeys.ARM_P_TERM, DEFAULT_ARM_P);
		double i = Robot.preferences.getDouble(PreferenceKeys.ARM_I_TERM, DEFAULT_ARM_I);
		double d = Robot.preferences.getDouble(PreferenceKeys.ARM_D_TERM, DEFAULT_ARM_D);
		armPIDControllerInit(p, i, d, setpoint, tolerance);
  }
  
  public void armAnglePIDExecute() {
		double currentPIDOutput = pidController.update(RobotMap.armEncoder.getDistance());

		SmartDashboard.putNumber("Arm Angle PID/Error", pidController.getError());
		SmartDashboard.putNumber("Arm Angle PID/Output", currentPIDOutput);

		rawMoveArm(currentPIDOutput);
	}

	public void armAnglePIDEnd() {
		pidController = null;
		stop();
	}
	
	public boolean armPIDControllerOnTarget() {
		return pidController.onTarget();
	}

	public boolean atFrontLimit () {
		return !RobotMap.armFrontLimitSwitch.get();
	}

	public boolean atBackLimit () {
		return !RobotMap.armBackLimitSwitch.get();
	}
}

