/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.utilities.PositionTracker;
import frc.robot.utilities.Waypoint;
import frc.robot.utilities.Waypoint.CoordinateType;

/**
 * Add your docs here.
 */
public class FollowWaypoints extends Command {
    private int waypointIndex;
	private double targetX, targetY;
	Waypoint currentWaypoint;
	private Waypoint[] waypoints;
	
	public FollowWaypoints(Waypoint[] waypoints) {
		this.waypoints = waypoints;
		this.waypointIndex = -1;
	}
	
	private void initializeNextWaypoint()
	{
		waypointIndex++;
		if (waypointIndex < waypoints.length) {
			currentWaypoint = waypoints[waypointIndex];
			if (currentWaypoint.getType() == CoordinateType.RELATIVE) {
				targetX += currentWaypoint.getX();
				targetY += currentWaypoint.getY();
			} else {
				targetX = currentWaypoint.getX();
				targetY = currentWaypoint.getY();
			}
			System.out.println("Next Waypoint is :" + targetX + ", " + targetY);
		}
	}
	
	public void initialize() {
		System.out.println("FollowWaypoints init");
		waypointIndex = -1;
		targetX = Robot.positionTracker.getX();
		targetY = Robot.positionTracker.getY();
		initializeNextWaypoint();
		Robot.drive.driveOnHeadingInit(Robot.positionTracker.calculateAngleTo(targetX, targetY));
	}
	
	public void execute() {
		if (currentWaypoint != null) {
			double newHeading = Robot.positionTracker.calculateAngleTo(targetX, targetY);
			System.out.print(String.format("X:%.1f Y:%.1f heading:%.1f ", Robot.positionTracker.getX(),
				Robot.positionTracker.getY(), newHeading));
			Robot.drive.driveOnHeadingExecute(currentWaypoint.getPower(), newHeading);
		}
	}
	

	public boolean isFinished() {
		if (currentWaypoint != null && currentWaypoint.isFinished()) {
			initializeNextWaypoint();
		}
		return waypointIndex >= waypoints.length;
	}
	
	protected void end() {
		System.out.println("FollowWaypoints end");
		Robot.drive.driveOnHeadingEnd();
	}
	
	protected void interrupted() {
		end();
	}

}
