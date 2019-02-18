package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Victor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into0
 * 
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  private static final int DRIVE_LEFT_ENCODER_PORT1 = 0; 
  private static final int DRIVE_LEFT_ENCODER_PORT2 = 1;
  private static final int DRIVE_RIGHT_ENCODER_PORT1 = 2;
  private static final int DRIVE_RIGHT_ENCODER_PORT2 = 3;
  private static final int ARM_ENCODER_PORT_1 = 4; // temp make sure these are correct ports
  private static final int ARM_ENCODER_PORT_2 = 5;
  private static final int ARM_FRONT_LIMIT_SWITCH_PORT = 6;
  private static final int ARM_BACK_LIMIT_SWITCH_PORT = 7;
  
  private static final double DRIVE_LEFT_ENCODER_DIST_PER_PULSE = 0.009703125;
  private static final double DRIVE_RIGHT_ENCODER_DIST_PER_PULSE = 0.009703125;

  public static WPI_VictorSPX driveFrontLeftMotor;
  public static WPI_VictorSPX driveMiddleLeftMotor;
  public static WPI_VictorSPX driveBackLeftMotor;
  public static WPI_VictorSPX driveFrontRightMotor;
  public static WPI_VictorSPX driveMiddleRightMotor;
  public static WPI_VictorSPX driveBackRightMotor;

  public static Victor armMotor;
  public static Victor cargoAcquirerMotor;
  public static Victor climberMotor;
  public static Encoder armEncoder;
  public static Encoder driveLeftEncoder;
  public static Encoder driveRightEncoder;

  public static DoubleSolenoid hatchClawSolenoid;
  public static DoubleSolenoid hatchExtensionSolenoid;
  public static DoubleSolenoid climberSolenoid;
  public static DoubleSolenoid gearboxSolenoid;

  public static DigitalInput armFrontLimitSwitch;
  public static DigitalInput armBackLimitSwitch;

  public static Compressor compressor;
  
  public static AHRS navx;

  public static void init() {
    driveMiddleLeftMotor = new WPI_VictorSPX(4);
    driveBackLeftMotor = new WPI_VictorSPX(5);
    driveFrontLeftMotor = new WPI_VictorSPX(6);
    driveFrontRightMotor = new WPI_VictorSPX(1);
    driveMiddleRightMotor = new WPI_VictorSPX(2);
    driveBackRightMotor = new WPI_VictorSPX(3);

    driveFrontRightMotor.setInverted(true);
    driveFrontLeftMotor.setInverted(true);
    driveMiddleLeftMotor.setInverted(true);
    driveMiddleRightMotor.setInverted(true);
    driveBackLeftMotor.setInverted(true);
    driveBackRightMotor.setInverted(true);

    climberMotor.setInverted(true);

    armMotor = new Victor(1);
    climberMotor = new Victor(2);
    cargoAcquirerMotor = new Victor(0);
    armEncoder = new Encoder(ARM_ENCODER_PORT_1, ARM_ENCODER_PORT_2);

    driveLeftEncoder = new Encoder(DRIVE_LEFT_ENCODER_PORT1, DRIVE_LEFT_ENCODER_PORT2);
    driveRightEncoder = new Encoder(DRIVE_RIGHT_ENCODER_PORT1, DRIVE_RIGHT_ENCODER_PORT2, true);
    driveLeftEncoder.setDistancePerPulse(DRIVE_LEFT_ENCODER_DIST_PER_PULSE);
    driveRightEncoder.setDistancePerPulse(DRIVE_RIGHT_ENCODER_DIST_PER_PULSE);

    hatchClawSolenoid = new DoubleSolenoid(0, 1);          // TODO CHANGE THE CHANNELS LATER
    hatchExtensionSolenoid = new DoubleSolenoid (2,3);
    climberSolenoid = new DoubleSolenoid(6, 7);
    gearboxSolenoid = new DoubleSolenoid(4, 5);

    armFrontLimitSwitch = new DigitalInput(ARM_FRONT_LIMIT_SWITCH_PORT);
    armBackLimitSwitch = new DigitalInput(ARM_BACK_LIMIT_SWITCH_PORT);

    compressor = new Compressor();
    compressor.start();

    navx = new AHRS(SPI.Port.kMXP); 
  }

  public static void resetSensors(){
    Robot.positionTracker.reset();
    navx.reset();
    armEncoder.reset();
    //TODO CHECK IF WE NEED TO RESET SOLENOIDS
    System.out.println("Sensors Reset");
  }  
}
