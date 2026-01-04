/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TELEOP 1", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx intake = null;
    private DcMotorEx shootR = null;
    private DcMotorEx shootL = null;
    private Servo gate = null;
    private Servo rgbLight;



    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fr");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "br");
        intake = hardwareMap.get(DcMotorEx.class, "int");
        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        gate = hardwareMap.get(Servo.class, "gate");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");




        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidfCoefficients= new PIDFCoefficients(16, 0, 0, 16);
        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shootL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        gate.setPosition(0.25);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double currentVelocity = shootR.getVelocity();
            if (currentVelocity >= 1560) {
                rgbLight.setPosition(0.5);
            } else {
                rgbLight.setPosition(0.280);
            }

            if (gamepad1.right_bumper){
                intake.setPower(1);
            }
            else if (gamepad1.left_bumper) {
                intake.setPower(0);
            }

            if (gamepad1.circle){
                shootR.setVelocity(1600);
                shootL.setVelocity(1600);
                gate.setPosition(0.0);
            } else if (gamepad1.cross) {
                shootR.setVelocity(0);
                shootL.setVelocity(0);
                gate.setPosition(0.25);
            }


            double max;
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Flywheel RPM", currentVelocity);
            telemetry.update();
        }
    }}
