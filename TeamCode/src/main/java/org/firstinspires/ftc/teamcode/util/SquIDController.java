package org.firstinspires.ftc.teamcode.util;

public class SquIDController {
    private double kSQ;
    public SquIDController(double kSQ) {
        this.kSQ = kSQ;
    }

    public double calculate(double error){
        return Math.sqrt(Math.abs(error * kSQ)) * Math.signum(error);
    }

    public void setkSQ(double kSQ) {
        this.kSQ =kSQ;
    }
}
