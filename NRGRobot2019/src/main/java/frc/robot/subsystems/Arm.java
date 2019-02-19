package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualMoveArm;
import frc.robot.commands.ManualMoveArmWithPID;
import frc.robot.utilities.PreferenceKeys;
import frc.robot.utilities.SimplePIDController;

/**
 * Subsystem, moves the arm.
 */
public class Arm extends Subsystem {
	/**
	 *
	 */

	private static final int DEAD_BAND_RANGE = 100;
	public static final double DEFAULT_ARM_MAX_POWER = 0.5;
	public static final double DEFAULT_ARM_P = 0.005;
	public static final double DEFAULT_ARM_I = DEFAULT_ARM_P / 10;
	public static final double DEFAULT_ARM_D = 0;

	public static final int DEFAULT_ARM_STOWED_TICKS = 0;
	public static final int DEFAULT_ARM_ACQUIRE_CARGO_TICKS = 235;
	public static final int DEFAULT_ARM_CARGO_SHIP_TICKS = 600;
	public static final int DEFAULT_ARM_ROCKET_CARGO_LOW_TICKS = 900;
	public static final int DEFAULT_ARM_ROCKET_CARGO_MEDIUM_TICKS = 1900;
	public static final int DEFAULT_ARM_ROCKET_CARGO_HIGH_TICKS = 1200;
	public static final int DEFAULT_ARM_MAX_ANGLE_TICKS = 2600; //slightly smaller than actual range (max = 2670)
	public static final int DEFAULT_ARM_TICK_TOLORANCE = 10; // TODO : figure out a good value line 21-26


  private SimplePIDController pidController;

	public enum Angle {
		
		ARM_STOWED_ANGLE(PreferenceKeys.ARM_STOWED_TICKS, DEFAULT_ARM_STOWED_TICKS),
		ARM_ACQUIRE_CARGO_ANGLE(PreferenceKeys.ARM_ACQUIRE_CARGO_TICKS, DEFAULT_ARM_ACQUIRE_CARGO_TICKS),
		ARM_CARGO_SHIP_ANGLE(PreferenceKeys.ARM_CARGO_SHIP_TICKS, DEFAULT_ARM_CARGO_SHIP_TICKS),
		ARM_ROCKET_CARGO_LOW_ANGLE(PreferenceKeys.ARM_ROCKET_CARGO_LOW_TICKS, DEFAULT_ARM_ROCKET_CARGO_LOW_TICKS),
		ARM_ROCKET_CARGO_MEDIUM_ANGLE(PreferenceKeys.ARM_ROCKET_CARGO_MEDIUM_TICKS, DEFAULT_ARM_ROCKET_CARGO_MEDIUM_TICKS),
		ARM_ROCKET_CARGO_HIGH_ANGLE(PreferenceKeys.ARM_ROCKET_CARGO_HIGH_TICKS, DEFAULT_ARM_ROCKET_CARGO_HIGH_TICKS),
		ARM_MAX_ANGLE(PreferenceKeys.ARM_MAX_ANGLE_TICKS, DEFAULT_ARM_MAX_ANGLE_TICKS);
		
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
    setDefaultCommand(new ManualMoveArmWithPID());
  }

  // Positive power means up from the starting (stowed) position.
  public void moveArm(double power){
    rawMoveArm(power);
  }
  
  private void rawMoveArm(double power) {
	  if (power < 0 && atBackLimit()) {
			power = 0;
		}
	  else if (atFrontLimit()) {
			power = 0;
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
    double maxPower = Robot.preferences.getDouble(PreferenceKeys.ARM_MAX_POWER, DEFAULT_ARM_MAX_POWER);
    pidController = new SimplePIDController(p, i, d, true)
                .setOutputRange(-maxPower, maxPower)
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

  public void armAnglePIDInit() {
	  Robot.arm.armAnglePIDInit(RobotMap.armEncoder.get(), Arm.DEFAULT_ARM_TICK_TOLORANCE);
  }
  
  public void armAnglePIDExecute() {
    int armTicks = RobotMap.armEncoder.get();
    double armPIDOutput = pidController.update(armTicks);

    //if arm is stowed, don't run PID
    if(armTicks < DEAD_BAND_RANGE && armPIDControllerOnTarget()) {
      armPIDOutput = 0;
    }
    rawMoveArm(armPIDOutput);

    SmartDashboard.putNumber("Arm Angle PID/Output", armPIDOutput);
		SmartDashboard.putNumber("Arm Angle PID/Error", pidController.getError());
		SmartDashboard.putNumber("Arm Angle PID/Setpoint", pidController.getSetpoint());
  }

	public void armAnglePIDEnd() {
		pidController = null;
		stop();
	}
	
	public boolean armPIDControllerOnTarget() {
		return pidController.onTarget();
	}

	public void setPIDOutputLimits(double maxSpeed) {
		double power = Math.min(Math.abs(maxSpeed), Robot.preferences.getDouble(PreferenceKeys.ARM_MAX_POWER, DEFAULT_ARM_MAX_POWER));
		pidController.setOutputRange(-power, power);
	}

	public boolean atFrontLimit () {
		return !RobotMap.armFrontLimitSwitch.get();
	}

	public boolean atBackLimit () {
		return !RobotMap.armBackLimitSwitch.get();
	}
	
	public int getSetpoint() {
		return (int)pidController.getSetpoint();
	}
  
  public void setSetpoint(int setpointInTicks) {
		if (setpointInTicks > 0 && setpointInTicks < DEAD_BAND_RANGE && setpointInTicks != Arm.Angle.ARM_STOWED_ANGLE.getTicks()) {
			setpointInTicks = DEAD_BAND_RANGE;
		}
    pidController.setSetpoint(setpointInTicks);
  }

  public int getCurrentArmPosition() {
    return RobotMap.armEncoder.get();
	}
	
}

