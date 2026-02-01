package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class ShooterConstraints {
    public static Pose GOAL_POSE_RED = new Pose(138,138);
    public static Pose GOAL_POSE_BLUE = new Pose(138,138).mirror();
    public static double SCORE_HEIGHT = 26;// INCHES
    public static double SCORE_ANGLE = Math.toRadians(-30);// RAD
    public static double PASSTHROUGH_POINT_RADIUS = 5;// INCHES
}
