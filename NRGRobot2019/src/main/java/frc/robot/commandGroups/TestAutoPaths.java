/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.utilities.NRGPreferences;

public class TestAutoPaths extends CommandGroup {
  public static final String DEFAULT_TEST_PATH = "LEFT_TO_CARGO_FRONT_LEFT_HATCH";

  public TestAutoPaths() {
    String pathname = NRGPreferences.StringPrefs.TEST_PATH_NAME.getValue();
    addSequential(new FollowPathWeaverFile("output/" + pathname + ".pf1.csv"));

  }
}
