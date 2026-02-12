package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LimeLightConstants {
    // Limelight mounting configuration
    public static double MOUNT_ANGLE_DEGREES = 22.5; // Angle back of limelight from vert
    public static double LENS_HEIGHT_INCHES = 12.55906;
    public static double GOAL_HEIGHT_INCHES = 29.5;  // Height of target, actual height of goal == 29.5, 19 == test
    public static double TURRET_KP = 0;
    public static double TURRET_KD = 0;
    public static double TURRET_KSTATIC = 0;
    public static double TURRET_MAX_POWER = 1;
    public static double Fly = 0;
    public static double Hood = 0;


    public static double TURRET_GEAR_RATIO = 1.111;
    public static double TURRET_MAX_ANGLE = 160.0;   // Max degrees right
    public static double TURRET_MIN_ANGLE = -160.0;  // Max degrees left
    public static double TURRET_DEADBAND = 3;
    public static double ENCODER_VOLTAGE_MAX = 3.3;
    public static double ENCODER_DEGREES_PER_VOLT = 360.0 / ENCODER_VOLTAGE_MAX;
    public static int DEFAULT_PIPELINE = 0;
    public static int POLL_RATE_HZ = 100;
}
