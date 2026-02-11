package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
public class LLSub {
    private Limelight3A limelight;
    private CRServo turretR;
    private CRServo turretL;
    private AnalogInput turretEncoder;

    public static double turretKp = 0.03;
    public static double turretMaxPower = 1.0;
    public static double LIMELIGHT_OFFSET = 7.42;
    public static double GOAL_X = 14.5;
    public static double GOAL_Y = 128.5;

    // --- State Variables ---
    private double turretOffsetDegrees = 0; // Calibration offset
    private double currentTurretAngle = 0;
    private double targetTurretAngle = 0;
    private double calculatedHoodAngle = 0;
    private double calculatedFlywheelVel = 0;
    private double goalDist = 0;
    private boolean hasValidTarget = false;

    private LinearOpMode opMode;

    public LLSub(LinearOpMode opMode) {
        this.opMode = opMode;

        // Hardware Map
        // Ensure "turretEncoder" is configured as an Analog Input device
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        turretR = opMode.hardwareMap.get(CRServo.class, "turretR");
        turretL = opMode.hardwareMap.get(CRServo.class, "turretL");
        turretEncoder = opMode.hardwareMap.get(AnalogInput.class, "turretEncoder");

        // Servo Config
        turretR.setDirection(CRServo.Direction.REVERSE);
        turretL.setDirection(CRServo.Direction.REVERSE);

        // Limelight Config
        opMode.telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        // Calibrate Turret Center
        // Assumes robot initializes with turret straight forward (0 deg)
        turretOffsetDegrees = getRawEncoderDegrees();
    }

    /**
     * Main update loop.
     * 1. Reads Limelight to relocalize PedroPathing (correct drift).
     * 2. Reads PedroPathing (which is now accurate) to aim Turret.
     * @param follower The main PedroPathing follower object from your OpMode
     */
    public void update(Follower follower) {
        LLResult result = limelight.getLatestResult();

        // ------------------------------------------
        // 1. Relocalization Logic (Limelight -> PedroPathing)
        // ------------------------------------------
        if (result != null && result.isValid()) {
            Pose3D botPose3D = result.getBotpose(); // Gets pose in Field-Space (usually Meters, Center-Origin)

            if (botPose3D != null) {
                double x_meters = botPose3D.getPosition().x;
                double y_meters = botPose3D.getPosition().y;

                // 1. Convert Meters to Inches
                double x_inches = x_meters * 39.3701;
                double y_inches = y_meters * 39.3701;

                // 2. Convert Center-Origin (Limelight) to Corner-Origin (PedroPathing)
                // Standard FTC field is 144x144 inches. Center is (0,0) for LL, (72,72) for Pedro.
                // NOTE: If your Limelight Map is already configured for Corner Origin, remove the +72.
                double pedroX = x_inches + 72.0;
                double pedroY = y_inches + 72.0;
                double pedroHeading = botPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

                // 3. Feed the corrected pose back into the Follower
                // This essentially "resets" the odometry to the vision truth
                follower.setPose(new Pose(pedroX, pedroY, pedroHeading));

                hasValidTarget = true;
            }
        } else {
            hasValidTarget = false;
        }

        // ------------------------------------------
        // 2. Field-Centric Turret Tracking
        // ------------------------------------------
        // We use the follower's pose (which is robust + corrected by vision)
        Pose robotPose = follower.getPose();

        // Calculate vector to goal
        double distX = GOAL_X - robotPose.getX();
        double distY = GOAL_Y - robotPose.getY();

        // Angle from robot to goal (Field Relative)
        double angleToGoalRad = Math.atan2(distY, distX);
        double angleToGoalDeg = Math.toDegrees(angleToGoalRad);

        // Convert to Turret Relative Angle (Robot Relative)
        // We subtract the robot's heading to know where the turret should look relative to the chassis
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        targetTurretAngle = MathFunctions.normalizeAngle(angleToGoalDeg - robotHeadingDeg);

        // ------------------------------------------
        // 3. Turret PID & Hardware Control
        // ------------------------------------------
        currentTurretAngle = getTurretAngle();
        double error = targetTurretAngle - currentTurretAngle;

        // Angle Wrap: ensure we take the shortest path (e.g. 170 to -170 is 20 deg, not 340)
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        double power = error * turretKp;

        // --- SOFT LIMITS (-160 to +160) ---
        // If at left limit (-160) and trying to go more left (negative power), stop.
        // If at right limit (+160) and trying to go more right (positive power), stop.
        if (currentTurretAngle > 160 && power > 0) {
            power = 0;
        } else if (currentTurretAngle < -160 && power < 0) {
            power = 0;
        }

        // Clamp Power
        power = Math.max(-turretMaxPower, Math.min(turretMaxPower, power));

        turretR.setPower(power);
        turretL.setPower(power);

        // ------------------------------------------
        // 4. Shooting Calculations
        // ------------------------------------------
        double centerToGoal = Math.hypot(distX, distY);
        goalDist = centerToGoal - LIMELIGHT_OFFSET;
        calculatedFlywheelVel = getFlywheelVelocity(goalDist);
        calculatedHoodAngle = getHoodAngle(goalDist);
    }

    /**
     * Converts Encoder Voltage to Degrees.
     * 0 degrees = position at code initialization.
     */
    private double getTurretAngle() {
        double rawDegrees = getRawEncoderDegrees();
        double relativeDegrees = rawDegrees - turretOffsetDegrees;

        // Normalize to -180 to 180
        while (relativeDegrees > 180) relativeDegrees -= 360;
        while (relativeDegrees <= -180) relativeDegrees += 360;

        return relativeDegrees;
    }

    // Safety stop
    public void stop() {
        limelight.stop();
        turretR.setPower(0);
        turretL.setPower(0);
    }

    private double getRawEncoderDegrees() {
        // Reads voltage from Axon/Lamprey encoder (0-3.3V)
        return (turretEncoder.getVoltage() / 3.3) * 360.0;
    }

    public static double getHoodAngle(double goalDist) {
        double rawAngle = (-1.74147E-7 * Math.pow(goalDist, 3))
                + (0.0000321043 * Math.pow(goalDist, 2))
                - (0.0034149 * goalDist)
                + 0.466667;
        return MathFunctions.clamp(rawAngle, 0.0, 0.9);
    }

    public static double getFlywheelVelocity(double goalDist) {
        double rawVelocity = (0.0145089 * Math.pow(goalDist, 2))
                + (0.533929 * goalDist)
                + 510.4;
        return MathFunctions.clamp(rawVelocity, 0.0, 2200.0);
    }

    // Getters
    public double getDistanceToGoal() { return goalDist; }
    public double getFinalFlywheelVel() { return calculatedFlywheelVel; }
    public double getFinalHoodAngle() { return calculatedHoodAngle; }
    public boolean hasValidTarget() { return hasValidTarget; }
}