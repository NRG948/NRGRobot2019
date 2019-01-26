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

import frc.robot.RobotMap;

public class PositionTracker {
    
    private double x;
    private double y;
    private double previousLeftEncoder;
    private double previousRightEncoder;
    
    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }

    public void setPosition(double x, double y){
        this.x = x;
        this.y = y;
        this.previousLeftEncoder = RobotMap.driveLeftEncoder.getDistance();
        this.previousRightEncoder = RobotMap.driveRightEncoder.getDistance();

    }

    public void updatePosition(){
        double currentLeftEncoder = RobotMap.driveLeftEncoder.getDistance();
        double currentRightEncoder = RobotMap.driveRightEncoder.getDistance();
        double leftDelta = currentLeftEncoder - this.previousLeftEncoder;
        double rightDelta = currentRightEncoder - this.previousRightEncoder;
        double distance = (leftDelta + rightDelta)/2;
        double heading = Math.toRadians(RobotMap.navx.getAngle());
        double deltaX = distance * Math.sin(heading);
        double deltaY = distance * Math.cos(heading);

        this.x += deltaX;
        this.y += deltaY;

        this.previousLeftEncoder = currentLeftEncoder;
        this.previousRightEncoder = currentRightEncoder;
    }

    public double calculateDistance(double xOrigin, double yOrigin){
        double deltaX = this.x - xOrigin;
        double deltaY = this.y - yOrigin;
        return Math.sqrt((deltaX*deltaX)+(deltaY*deltaY));
    }

    public double calculateAngleTo(double targetX, double targetY){
        double dX = targetX - this.x;
		double dY = targetY - this.y;
        double heading = Math.toDegrees(Math.atan2(dX, dY));
        return heading;
    }


}
