package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelPIDFTune extends OpMode{
    private DcMotorEx shootR = null;
    private DcMotorEx shootL = null;
    public double  ShootSpeed = 2000;
    public double idle = 900;
    double currentTargVel = ShootSpeed;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    @Override
    public void init(){
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        shootR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients= new PIDFCoefficients(P, 0, 0, F);
        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shootL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init done");
    }

    @Override
    public void loop(){
        if (gamepad1.triangleWasPressed()){
            if (currentTargVel == ShootSpeed){
                currentTargVel = idle;
            }else {currentTargVel = ShootSpeed;}
        }
        if (gamepad1.circleWasPressed()){
            stepIndex = (stepIndex+1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            P -= stepSizes[stepIndex];
        }
        //set new pidf
        PIDFCoefficients pidfCoefficients= new PIDFCoefficients(P, 0, 0, F);
        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shootL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        shootR.setVelocity(currentTargVel);
        shootL.setVelocity(currentTargVel);

        double currentVelocity = shootR.getVelocity();
        double error = currentTargVel - currentVelocity;

        telemetry.addData("Target Velocity", currentTargVel);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R", F);
        telemetry.addData("Step Sizes", "%.4f (Circle Button)", stepSizes[stepIndex]);





    }
}
