package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Limelight3A")
public class LimeLight3A extends LinearOpMode {

    private Limelight3A limelight;
    private Servo rgbLight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        rgbLight = hardwareMap.get(Servo.class,"rgbLight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {

                double ta = result.getTa();

                // FIXED: Logical condition
                // Example: Turn GREEN when target area is between 6 and 8.5
                // Adjust these values based on your actual needs
                if (ta >= 6.0 && ta <= 8.5) {
                    rgbLight.setPosition(0.5);
                    telemetry.addData("LED", rgbLight.getPosition());
                } else {
                    rgbLight.setPosition(0.280);
                    telemetry.addData("LED", rgbLight.getPosition());
                }
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}