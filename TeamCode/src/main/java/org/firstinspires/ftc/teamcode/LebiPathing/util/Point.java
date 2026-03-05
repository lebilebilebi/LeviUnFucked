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

    public enum HeadingMode {
        CONSTANT, // Turn to target angle immediately using PID
        LINEAR    // Gradually turn to target angle based on distance traveled
    }

    @Getter @Setter
    private HeadingMode headingMode = HeadingMode.CONSTANT;
    /**
     * A value between 0.0 and 1.0 representing path completion %
     * 1.0 = Finish turning exactly when robot reaches target
     * 0.5 = Finish turning when robot is halfway to target
     * 0.1 = Finish turning within the first 10% of movement
     * NOTE: This value is IGNORED if headingMode is CONSTANT.
     */
    @Getter @Setter
    private double headingEndTime = 1.0;

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

    public Point setHeadingInterpolation(HeadingMode mode, double turnEnd) {
        this.headingMode = mode;
        this.headingEndTime = turnEnd;
        return this;
    }

    public Point setHeadingInterpolation(HeadingMode mode) {
        return setHeadingInterpolation(mode, 1.0);
    }
}