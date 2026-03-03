package org.firstinspires.ftc.teamcode.LebiPathing.util;

import lombok.Getter;
import lombok.Setter;
public class Point {
    @Getter
    @Setter
    private double x, y, heading;

    public Point (double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Point(){
        this.x = 0;
        this.y = 0;
        this.heading =0;
    }

    public void setPoint (double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}