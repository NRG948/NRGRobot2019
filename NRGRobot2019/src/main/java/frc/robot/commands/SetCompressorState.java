/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotMap;

public class SetCompressorState extends InstantCommand {

  public enum CompressorState {
    ENABLED, DISABLED;
  }

  private CompressorState state;

  public SetCompressorState(CompressorState state) {
    this.state = state;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (state == CompressorState.ENABLED) {
      RobotMap.compressor.start();
    } else {
      RobotMap.compressor.stop();
    }
  }
}
