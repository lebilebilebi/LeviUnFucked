package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limelight3A_PD_Tracking")
public class NewLimeLight3A extends LinearOpMode {

    private Limelight3A limelight;
    private Servo rgbLight;
    private Servo gate;
    private DcMotorEx turretMotor;
    private DcMotorEx flywheelR;
    private DcMotorEx flywheelL;

    // TWEAKER PREVENTION MECHANISM (TUNE p for speed and d for breaking)
    private static final double speedish = 0.026;
    private static final double tweakerPreventionSystem2000 = 0.016;
    private static final double TURRET_SPEED_LIMIT = 0.8;
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        rgbLight = hardwareMap.get(Servo.class,"rgbLight");
        gate = hardwareMap.get(Servo.class,"gate");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelL = hardwareMap.get(DcMotorEx.class, "flyL");

        flywheelL.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelR.setDirection(DcMotorEx.Direction.REVERSE);


        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        lastError = 0;

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();
                double currentError = -tx;
                double proportional = currentError * speedish;
                double derivative = (currentError - lastError) * tweakerPreventionSystem2000;
                lastError = currentError;
                double drive = proportional + derivative;

                if (drive > TURRET_SPEED_LIMIT) {
                    drive = TURRET_SPEED_LIMIT;
                } else if (drive < -TURRET_SPEED_LIMIT) {
                    drive = -TURRET_SPEED_LIMIT;
                }

                turretMotor.setPower(drive);

                double ta = result.getTa();
                if (ta >= 3.0 && ta <= 8.5) {
                    rgbLight.setPosition(0.5);
                    gate.setPosition(0);
                    flywheelL.setPower(0.9);
                    flywheelR.setPower(0.9);

                } else {
                    rgbLight.setPosition(0.280);
                    gate.setPosition(0.5);
                    flywheelL.setPower(0);
                    flywheelR.setPower(0);
                }

                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                telemetry.addData("TAG STATUS: NONZERO, ID:", result.getFiducialResults().get(0).getFiducialId());
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("tx", tx);
                telemetry.addData("ta (distance)",ta);
                telemetry.addData("Motor Power", drive);
                telemetry.addData("Vel thingy value (TUNE THIS PLZZZ)", speedish);
                telemetry.addData("breaking value (TUNE THIS ALSO)", tweakerPreventionSystem2000);
                telemetry.addData("LED", rgbLight.getPosition());


            } else {
                turretMotor.setPower(0);
                telemetry.addData("TAG STATUS", "VOID");
                lastError = 0;
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
