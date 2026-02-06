package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@TeleOp(name="TELEOP 1", group="Linear OpMode")
public class DECODEOPMODE1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    InternalMechanisms mechanisms;
    DriveMechanisms drive;
    @Override
    public void runOpMode() {
        mechanisms = new InternalMechanisms(hardwareMap);
        drive = new DriveMechanisms(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            mechanisms.update();

            if (gamepad2.cross) {
                mechanisms.setState(RoboStates.AUTO_SCORE);
            } else if (gamepad2.right_bumper) {
                mechanisms.setState(RoboStates.INTAKE);
            } else if (gamepad2.left_bumper) {
                mechanisms.setState(RoboStates.IDLE);
            }

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            drive.drive(axial, lateral, yaw);

            telemetry.addData("Stage 1/2 Current", "%.1f / %.1f A", mechanisms.getStage1Current(), mechanisms.getStage2Current());
            telemetry.update();
        }
    }
}