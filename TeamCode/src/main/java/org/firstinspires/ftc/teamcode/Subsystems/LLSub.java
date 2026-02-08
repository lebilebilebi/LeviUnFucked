package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LLSub {
    private Limelight3A limelight;
    private CRServo turretR;
    private CRServo turretL;

    public static double kP = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double deadband = LimeLightConstants.TURRET_DEADBAND;

    private double lastError = 0;
    private double calculatedHoodAngle = 0;
    private double calculatedFlywheelVel = 0;
    private double goalDist = 0;
    private double currentTx = 0;
    private double currentTy = 0;
    private boolean hasValidTarget = false;

    private final ElapsedTime timer = new ElapsedTime();
    private LinearOpMode opMode;
    
    public LLSub(LinearOpMode opMode) {
        this.opMode = opMode;
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        turretR = opMode.hardwareMap.get(CRServo.class, "turretR");
        turretL = opMode.hardwareMap.get(CRServo.class, "turretL");

        turretR.setDirection(CRServo.Direction.REVERSE);
        turretL.setDirection(CRServo.Direction.REVERSE);

        opMode.telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update() {
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasValidTarget = true;
            currentTx = result.getTx();
            currentTy = result.getTy();
            double error = currentTx;

            // Turret tracking
            double pOut = error * kP;
            double dOut = (error - lastError) * kD;
            double fOut = 0;
            if (Math.abs(error) > 0.5) {
                fOut = Math.signum(error) * kF;
            }

            double power = pOut + dOut + fOut;
            turretR.setPower(power);
            turretL.setPower(power);
            lastError = error;

            // Distance calculation
            double limelightMountAngleDegrees = LimeLightConstants.MOUNT_ANGLE_DEGREES;
            double limelightLensHeightInches = LimeLightConstants.LENS_HEIGHT_INCHES;
            double goalHeightInches = LimeLightConstants.GOAL_HEIGHT_INCHES;

            double angleToGoalDegrees = limelightMountAngleDegrees + currentTy;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            goalDist = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            // Calculate flywheel and hood values
            calculatedFlywheelVel = getFlywheelVelocity(goalDist);
            calculatedHoodAngle = getHoodAngle(goalDist);

        } else {
            hasValidTarget = false;
            lastError = 0;
            turretR.setPower(0);
            turretL.setPower(0);
        }
    }

    public void stop() {
        limelight.stop();
        turretR.setPower(0);
        turretL.setPower(0);
    }

    public void reset() {
        timer.reset();
        lastError = 0;
        hasValidTarget = false;
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

    public double getFinalFlywheelVel() {
        return calculatedFlywheelVel;
    }

    public double getFinalHoodAngle() {
        return calculatedHoodAngle;
    }

    public double getDistanceToGoal() {
        return goalDist;
    }

    public boolean hasValidTarget() {
        return hasValidTarget;
    }

    public double getTx() {
        return currentTx;
    }

    public double getTy() {
        return currentTy;
    }
}
