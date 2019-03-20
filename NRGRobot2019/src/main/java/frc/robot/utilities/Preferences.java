/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import frc.robot.Robot;

/**
 * Defines enums for all robot preferences.
 */
public class Preferences {

    
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
        PATH_D_TERM("PathD", 0.0),
        
        ARM_P_TERM("ArmP", 0.01),
        ARM_I_TERM("ArmI", 0.002),
        ARM_D_TERM("ArmD", 0.001),
        ARM_MAX_POWER("ArmMaxPower", 0.50),s
        
  
  //public static final int DEFAULT_ARM_TICK_TOLORANCE = 10; // TODO : figure out a good value line 21-26
  
        ARM_STOWED_TICKS("ArmStowedTicks", 0),
        ARM_ACQUIRE_CARGO_TICKS("ArmAcquireCargoTicks", 280),
        ARM_CARGO_SHIP_TICKS("ArmCargoShipTicks", 970),
        ARM_ROCKET_CARGO_LOW_TICKS("ArmRocketCargoLowTicks", 700),
        ARM_ROCKET_CARGO_MEDIUM_TICKS("ArmRocketCargoMediumTicks", 1260),
        ARM_MAX_ANGLE_TICKS("ArmMaxAngleTicks", 2600), //TODO : figure out latest max value
        ARM_INVERSION_TICKS("ArmInversionTicks", 1680),
        ARM_HATCH_MEDIUM_TICKS("ArmHatchMediumTicks", 1800), // TBD
        ARM_LEVEL_TICKS("ArmLevelTicks", 750);
        
        // DRIVE_TO_VISION_TAPE_MIN_POWER("VisionMinPower";
        // DRIVE_TO_VISION_TAPE_MAX_POWER("VisionMaxPower";
        
        // CLIBMER_REAR_POWER("ClimberRearPower";
        // CLIMBER_ARMS_POWER("ClimberArmsPower";
        
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
    
    public enum BooleanPrefs {
        
        USING_PRACTICE_BOT("UsingPracticeBot", false),
        PATHS_SQUARE_INPUTS("PathsSquareInputs", false),
        TURN_SQUARE_INPUTS("TurnSquareInputs", true),
        TELEOP_SQUARE_INPUTS("TeleopSquareInputs", true),
        DRIVE_SQUARE_INPUTS("DriveSquareInputs", true);
        
        private String key;
        private boolean defaultValue;
        
        BooleanPrefs(String key, boolean defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
        }
        
        public String getKey() {
            return key;
        }
        
        public boolean getValue() {
            return Robot.preferences.getBoolean(key, defaultValue);
        }
    }
    
    public enum StringPrefs {
        
        TEST_PATH_NAME("TestPathName", "LEFT_TO_CARGO_FRONT_LEFT_HATCH");
        
        public static final String WRITE_DEFAULT = "WriteDefault";
        private String key;
        private String defaultValue;
        
        StringPrefs(String key, String defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
        }

        public String getKey() {
            return key;
        }

        public String getValue() {
            return Robot.preferences.getString(key, defaultValue);
        }
    }
}
