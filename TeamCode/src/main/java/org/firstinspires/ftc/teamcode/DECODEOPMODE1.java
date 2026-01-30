
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@TeleOp(name="TELEOP 1", group="Linear OpMode")
public class DECODEOPMODE1 extends LinearOpMode {
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

        while (opModeIsActive()) {
            mechanisms.update();
            if (gamepad2.crossWasPressed()) {
                mechanisms.setState(RoboStates.AUTO_SCORE);
            } else if (gamepad2.right_bumper) {
                mechanisms.setState(RoboStates.INTAKE);
            } else if (gamepad2.left_bumper){
                mechanisms.setState(RoboStates.IDLE);
            } else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                mechanisms.setState(RoboStates.FULL_IDLE);
            }
            double axial = -gamepad2.left_stick_y;
            double lateral = gamepad2.left_stick_x;
            double yaw = gamepad2.right_stick_x;
            drive.drive(axial, lateral, yaw);

            // TELEMETRY DATA
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