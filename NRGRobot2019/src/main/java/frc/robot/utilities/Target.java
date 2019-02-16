package frc.robot.utilities;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

public class Target {
    public enum Side{LEFT, RIGHT, UNKOWN;}
    private Point minX;
    private Point maxX;
    private Point minY;
    private Point maxY;
    private Side side;
    
    public Target(MatOfPoint mat) {
        Point[] points = mat.toArray();
        minX = points[0];
        minX.y = -minX.y;
        maxX = points[0];
        maxX.y = -maxX.y;
        minY = points[0];
        minY.y = -minY.y;
        maxY = points[0];
        maxY.y = -maxY.y;
        for(Point p: points) {
            p.y = -p.y;
            if(p.x < minX.x) {
                minX = p;
            }
            if(p.y < minY.y) {
                minY = p;
            }
            if(p.x > maxX.x) {
                maxX = p;
            }
            if(p.y < maxY.y) {
                maxY = p;
            }
        }

        if(minX.y > maxX.y) {
            side = Side.RIGHT;

        }
        else if(minX.y < maxX.y) {
            side = Side.LEFT;
        } else {
            side = Side.UNKOWN;
        }
    }
    public String toString(){
        return "Target: "+side.toString();
    }
    public Side getSide(){
        return this.side;
    }
    public Point getMinX(){
        return this.minX;
    }

    public Point getMinY() {
        return this.minY;
    }

    public Point getMaxX() {
        return this.maxX;
    }

    public Point getMaxY() {
        return this.maxY;
    }

    public Point getCenter() {
        return new Point((getMinX().x + getMaxX().x)/2, (getMinY().y + getMaxY().y)/2);
    }
}