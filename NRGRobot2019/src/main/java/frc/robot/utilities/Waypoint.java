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
public class Waypoint {
    private double x, y, power;
	private CoordinateType type;

	private WaypointPredicate waypointPredicate;

	public Waypoint(double x, double y, CoordinateType type, double power, WaypointPredicate waypointPredicate) {
		this.x = x;
		this.y = y;
		this.type = type;
		this.power = power;
		this.waypointPredicate = waypointPredicate;
	}
	
	public enum CoordinateType {
		ABSOLUTE, RELATIVE;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public CoordinateType getType() {
		return type;
	}

	public double getPower() {
		return power;
	}

	public boolean isFinished() {
		return waypointPredicate.isAtWaypoint(this);
	}
	
	public static class GreaterThanY implements WaypointPredicate {
		@Override
		public boolean isAtWaypoint(Waypoint w) {
			return Robot.positionTracker.getY() > w.getY();
		}
	}
	
	public static class GreaterThanX implements WaypointPredicate {
		@Override
		public boolean isAtWaypoint(Waypoint w) {
			return Robot.positionTracker.getX() > w.getX();
		}
	}
	
	public static class LessThanX implements WaypointPredicate {
		@Override
		public boolean isAtWaypoint(Waypoint w) {
			return Robot.positionTracker.getX() < w.getX();
		}
	}
	
	public static class WithinInches implements WaypointPredicate {
		private double tolerance;

		public WithinInches(double tolerance) {
			this.tolerance = tolerance;
		}
		
		@Override
		public boolean isAtWaypoint(Waypoint w) {
			return Robot.positionTracker.calculateDistance(w.getX(), w.getY()) <= tolerance;
		}
	}

}
