/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Preferences {

    private static final boolean DEFAULT_DRIVE_SQUARE_INPUTS = true;
    private static final boolean DEFAULT_TURN_SQUARE_INPUTS = true;
    private static final boolean DEFAULT_PATHS_SQUARE_INPUTS = true;
    public static final String WRITE_DEFAULT = "WriteDefault";
    public static final String TURN_SQUARE_INPUTS = "TurnSquareInputs";
    public static final String TELEOP_SQUARE_INPUTS = "TeleopSquareInputs";
    public static final String DRIVE_SQUARE_INPUTS = "DriveSquareInputs";
    public static final String PATHS_SQUARE_INPUTS = "PathsSquareInputs";
    
    public enum NumberPrefs {
        DRIVE_P_TERM("DriveP", 0.081),
        DRIVE_I_TERM("DriveI", 0.00016),
        DRIVE_D_TERM("DriveD", 0.0072),
        
        TURN_P_TERM("TurnP", 0.081),
        TURN_I_TERM("TurnI", 0.00016),
        TURN_D_TERM("TurnD", 0.0072),
        
        DISTANCE_DRIVE_P_TERM("DistanceDriveP", 0.04),
        DISTANCE_DRIVE_I_TERM("DistanceDriveI", 0.042),
        DISTANCE_DRIVE_D_TERM("DistanceDriveD", 0.0025),
        DISTANCE_TOLERANCE("DistanceTolerance", 0.75),

        PATH_P_TERM("PathP", 0.1),
        PATH_I_TERM("PathI", 0.0),
        PATH_D_TERM("PathD", 0.0);
        // PATHS_SQUARE_INPUTS("PathsSquareInputs";
      
        // ARM_P_TERM("ArmP";
        // ARM_I_TERM("ArmI";
        // ARM_D_TERM("ArmD";
        // ARM_MAX_POWER("ArmMaxPower";
      
        // ARM_STOWED_TICKS("ArmStowedTicks";
        // ARM_ACQUIRE_CARGO_TICKS("ArmAcquireCargoTicks";
        // ARM_CARGO_SHIP_TICKS("ArmCargoShipTicks";
        // ARM_ROCKET_CARGO_LOW_TICKS("ArmRocketCargoLowTicks";
        // ARM_ROCKET_CARGO_MEDIUM_TICKS("ArmRocketCargoMediumTicks";
        // ARM_MAX_ANGLE_TICKS("ArmMaxAngleTicks";
        // ARM_INVERSION_TICKS("ArmInversionTicks";
        // ARM_HATCH_MEDIUM_TICKS("ArmHatchMediumTicks";
        // ARM_LEVEL_TICKS("ArmLevelTicks";
      
        // DRIVE_TO_VISION_TAPE_MIN_POWER("VisionMinPower";
        // DRIVE_TO_VISION_TAPE_MAX_POWER("VisionMaxPower";
      
        // TEST_PATH_NAME("TestPathName";
      
        // CLIBMER_REAR_POWER("ClimberRearPower";
        // CLIMBER_ARMS_POWER("ClimberArmsPower";
      
        // USING_PRACTICE_BOT("UsingPracticeBot";

        private String key;
        private double defaultValue;

        NumberPrefs(String key, double defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
        }

        public String getKey() {
            return key;
        }

        public double getValue() {
            return Robot.preferences.getDouble(key, defaultValue);
        }
    }
    void a () {

    }
}
