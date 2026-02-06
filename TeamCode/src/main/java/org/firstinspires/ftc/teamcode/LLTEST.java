package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLightConstants;

@Configurable
@TeleOp(name = "Limelight3A_PD_Tracking_Fixed")
public class LLTEST extends LinearOpMode {
    InternalMechanisms mechanisms;

    LimeLightConstants limeLightConstants;

    private Limelight3A limelight;
    private Servo rgbLight;
    private Servo gate;
    private CRServo turretR;
    private CRServo turretL;

    public static double kP = LimeLightConstants.TURRET_KP;
    public static double kD = LimeLightConstants.TURRET_KD;
    public static double kF = LimeLightConstants.TURRET_KSTATIC;
    public static double deadband = LimeLightConstants.TURRET_DEADBAND;
    public static double maxPower = LimeLightConstants.TURRET_MAX_POWER;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretR = hardwareMap.get(CRServo.class, "turretR");
        turretL = hardwareMap.get(CRServo.class, "turretL");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        gate = hardwareMap.get(Servo.class, "gate");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        timer.reset();
        lastError = 0;

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double currentError = -tx;

                if (Math.abs(currentError) < deadband) currentError = 0;
                double pTerm = currentError * kP;
                double derivative = (currentError - lastError);
                double dTerm = derivative * kD;
                double fTerm = Math.signum(currentError) * kF;

                double drive = pTerm + dTerm + fTerm;

                drive = Math.max(-maxPower, Math.min(maxPower, drive));

                turretR.setPower(drive);
                turretL.setPower(drive);

                // Save state
                lastError = currentError;
                double limelightMountAngleDegrees = LimeLightConstants.MOUNT_ANGLE_DEGREES;

                double limelightLensHeightInches = LimeLightConstants.LENS_HEIGHT_INCHES;

                double goalHeightInches = LimeLightConstants.GOAL_HEIGHT_INCHES;

                double angleToGoalDegrees = limelightMountAngleDegrees + ty;

                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                double calculatedHoodAngle = getHoodAngle(distanceFromLimelightToGoalInches);
                double calculatedFlywheelVel = getFlywheelVelocity(distanceFromLimelightToGoalInches);

                telemetry.addData("TAG STATUS: NONZERO, ID:", result.getFiducialResults().get(0).getFiducialId());
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Servo Power", drive);
                telemetry.addData("Hood Angle", calculatedHoodAngle);
                telemetry.addData("Flywheel", calculatedFlywheelVel);
                telemetry.addData("Distance to goal", distanceFromLimelightToGoalInches);

            } else {
                turretR.setPower(0);
                turretL.setPower(0);
                telemetry.addData("TAG STATUS", "VOID");
                lastError = 0;
                kD = 0;
                kP =0;
            }
            telemetry.update();
        }
        limelight.stop();
    }
    public static double getHoodAngle(double goalDist) {
        double rawAngle = (0.00000120563 * Math.pow(goalDist, 3))
                - (0.000235615 * Math.pow(goalDist, 2))
                + (0.00615079 * goalDist)
                + 0.57;
        return MathFunctions.clamp(rawAngle, 0.0, 0.9);
    }
    public static double getFlywheelVelocity(double goalDist) {
        double rawVelocity = (0.000331549 * Math.pow(goalDist, 3))
                - (0.0542535 * Math.pow(goalDist, 2))
                + (8.92361 * goalDist)
                + 1212.0;
        return MathFunctions.clamp(rawVelocity, 0.0, 2200.0);
    }
}