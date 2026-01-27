package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@TeleOp(name="TELEOP 1", group="Linear OpMode")
public class DECODEOPMODE1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    InternalMechanisms mechanisms;
    DriveMechanisms drive;
    private Servo hoodR = null;
    private Servo hoodL = null;



    @Override
    public void runOpMode() { //_______________________________________
        mechanisms = new InternalMechanisms(hardwareMap);
        drive = new DriveMechanisms(hardwareMap);
        telemetry.addData("Status", "GOOOOOOOOOOOOOOOOOO!!!!!!!!!!!!!");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL = hardwareMap.get(Servo.class, "hoodL");
        telemetry.update();


        waitForStart(); //_______________________________________

        while (opModeIsActive()) { //_______________________________________
            mechanisms.update();
            if (runtime.time()==1.30){
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }
            if (gamepad2.right_bumper){
                mechanisms.setState(RoboStates.INTAKE);
            }
            else if (gamepad2.left_bumper){
                mechanisms.setState(RoboStates.IDLE);

            }
            else if (gamepad2.circleWasPressed()){
                mechanisms.setState(RoboStates.FAR_SIDE);
            }
            else if (gamepad2.crossWasPressed()){
                mechanisms.setState(RoboStates.CLOSE_SIDE);
            }
            else if (gamepad2.triangleWasPressed()){
                mechanisms.setState(RoboStates.AUTO_SCORE);
            }

            if(gamepad1.dpadUpWasPressed()){
                hoodR.setPosition(0);
                hoodL.setPosition(0);
            }
            else if(gamepad1.dpadDownWasPressed()){
                hoodR.setPosition(0.9);
                hoodL.setPosition(0.9);
            }


            //DRIVE
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            drive.drive(axial, lateral, yaw);
            //TELEMETRY DATA
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FLYWHEEL TPR", mechanisms.getVelocity());
            telemetry.addData("LED POSITION", mechanisms.getLedColor());
            telemetry.addData("Stage 1 current", mechanisms.getStage1Current());
            telemetry.addData("Stage 2 current", mechanisms.getStage2Current());
            telemetry.update();
        }
    }
}