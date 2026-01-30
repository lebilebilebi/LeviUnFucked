
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@TeleOp(name="SHOOTER TUNE", group="Linear OpMode")
public class SHOOTTUNE extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    InternalMechanisms mechanisms;
    DriveMechanisms drive;


    @Override
    public void runOpMode() { //_______________________________________
        mechanisms = new InternalMechanisms(hardwareMap);
        drive = new DriveMechanisms(hardwareMap);
        telemetry.addData("Status", "GOOOOOOOOOOOOOOOOOO!!!!!!!!!!!!!");
        telemetry.update();


        waitForStart(); //_______________________________________

        while (opModeIsActive()) { //_______________________________________
            mechanisms.update();

            if (mechanisms.getStage2Current() >= mechanisms.getStageTwoCurrentLimit()) {
                mechanisms.setState(RoboStates.FIRST_BALL_STOP);

            } else if (mechanisms.getStage1Current() >= mechanisms.getStageOneCurrentLimit()) {
                mechanisms.setState(RoboStates.FULL_STOP);

            } else if (gamepad2.right_bumper) {
                mechanisms.setState(RoboStates.INTAKE);

            } else if (gamepad2.left_bumper) {
                mechanisms.setState(RoboStates.IDLE);

            } else if (gamepad2.dpadUpWasPressed()) {
                mechanisms.increaseFlywheelVelocity(50);
            } else if (gamepad2.dpadDownWasPressed()) {
                mechanisms.decreaseFlywheelVelocity(50);
            } else if (gamepad2.dpadLeftWasPressed()) {
                mechanisms.decreaseHoodPosition(0.05);
            } else if (gamepad2.dpadRightWasPressed()) {
                mechanisms.increaseHoodPosition(0.05);
            } else if (gamepad1.dpadRightWasPressed()) {
                mechanisms.setState(RoboStates.IDLE);
            } else if (gamepad1.dpadDownWasPressed()) {
                mechanisms.setState(RoboStates.SHOOT);
            }


            //DRIVE
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            drive.drive(axial, lateral, yaw);
            //TELEMETRY DATA
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FLYWHEEL VELOCITY", mechanisms.getFlywheelVelocity());
            telemetry.addData("HOOD POSITION", mechanisms.getHoodPosition());
            telemetry.addData("FLYWHEEL TPR", mechanisms.getVelocity());
            telemetry.addData("LED POSITION", mechanisms.getLedColor());
            telemetry.addData("Stage 1 current", mechanisms.getStage1Current());
            telemetry.addData("Stage 2 current", mechanisms.getStage2Current());
            telemetry.update();
        }
    }
}