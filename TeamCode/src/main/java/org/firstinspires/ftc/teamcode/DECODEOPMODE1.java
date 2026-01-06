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

        while (opModeIsActive()) { //_______________________________________
            mechanisms.update();

            if (gamepad1.right_bumper){
                mechanisms.setState(RoboStates.INTAKE);
            }
            else if (gamepad1.left_bumper){
                mechanisms.setState(RoboStates.IDLE);

            }
            else if (gamepad1.circleWasPressed()){
                mechanisms.setState(RoboStates.FAR_SIDE);
            }
            else if (gamepad1.crossWasPressed()){
                mechanisms.setState(RoboStates.CLOSE_SIDE);
            }

            //DRIVE
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            drive.drive(axial, lateral, yaw);
            //TELEMETRY DATA
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flywheel RPM", mechanisms.getVelocity());
            telemetry.addData("LED POSITION ", mechanisms.getLedColor());
            telemetry.update();
        }
    }
}