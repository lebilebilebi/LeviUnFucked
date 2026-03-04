package org.firstinspires.ftc.teamcode.LebiPathing.util;

import lombok.Getter;
import lombok.Setter;
public class Point {
    @Getter
    @Setter
    private double x, y, heading;
    @Getter
    @Setter
    private double maxPower;
    @Getter
    @Setter
    private double radiusThresh = 3;
    @Getter
    @Setter
    private double headingThresh = 15;

    public Point (double x, double y, double heading, double maxPower){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.maxPower = maxPower;
    }

    public Point (double x, double y, double heading){
        this(x, y, heading, 1.0);
    }

    public Point(){
        this(0, 0, 0, 1.0);
    }

    public void setPoint (double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Point setThresholds(double radiusThresh_in, double headingThresh_deg) {
        this.radiusThresh = radiusThresh_in;
        this.headingThresh = headingThresh_deg;
        return this;
    }
}