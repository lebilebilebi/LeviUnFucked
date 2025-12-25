package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limelight3A_Tracking")
public class LimeLight3A extends LinearOpMode {

    private Limelight3A limelight;
    private Servo rgbLight;
    private DcMotorEx turretMotor;
    private static final double Kp = 0.03; // Proportional constant (needs tuning)
    private static final double TURRET_SPEED_LIMIT = 0.5; //jitter prevention (needs tuning)

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        rgbLight = hardwareMap.get(Servo.class,"rgbLight");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
             telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                     status.getTemp(), status.getCpu(),(int)status.getFps());
             telemetry.addData("Pipeline", "Index: %d, Type: %s",
                     status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {

                double tx = result.getTx();
                double headingError = -tx;

                double drive = headingError * Kp;

                if (drive > TURRET_SPEED_LIMIT) {
                    drive = TURRET_SPEED_LIMIT;
                } else if (drive < -TURRET_SPEED_LIMIT) {
                    drive = -TURRET_SPEED_LIMIT;
                }

                turretMotor.setPower(drive);

                double ta = result.getTa();

                if (ta >= 6.0 && ta <= 8.5) {
                    rgbLight.setPosition(0.5);
                } else {
                    rgbLight.setPosition(0.280);
                }
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                telemetry.addData("TAG STATUS: NONZERO, ID:", limelight.getLatestResult().getFiducialResults().get(0).getFiducialId());
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("tx", tx);
                telemetry.addData("ta", ta);
                telemetry.addData("Motor Power", drive);
                telemetry.addData("LED", rgbLight.getPosition());
            }
                else {
                    turretMotor.setPower(0);
                    telemetry.addData("TAG STATUS", "VOID");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
