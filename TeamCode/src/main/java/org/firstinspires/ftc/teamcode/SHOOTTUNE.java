
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimeLightConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;


@TeleOp(name="SHOOTER TUNE", group="Linear OpMode")
public class SHOOTTUNE extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    DriveMechanisms drive;
    Intake intake;
    Shooter shooter;
    LimeLightConstants limeLightConstants;
    public DcMotorEx shootR = null;
    public DcMotorEx shootL = null;
    private Servo gate = null;
    private Servo rgbLight = null;
    private Servo hoodR = null;
    private Servo hoodL = null;
    private DcMotorEx intakeStage1 = null;
    private DcMotorEx intakeStage2 = null;


    @Override
    public void runOpMode() { //_______________________________________
        drive = new DriveMechanisms(hardwareMap);
        telemetry.addData("Status", "GOOOOOOOOOOOOOOOOOO!!!!!!!!!!!!!");
        telemetry.update();

        intakeStage1 = hardwareMap.get(DcMotorEx.class, "int1");
        intakeStage2 = hardwareMap.get(DcMotorEx.class, "int2");

        intakeStage1.setDirection(DcMotorEx.Direction.FORWARD);
        intakeStage2.setDirection(DcMotorEx.Direction.REVERSE);

        intakeStage1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeStage2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shootR = hardwareMap.get(DcMotorEx.class, "shootR");
        shootL = hardwareMap.get(DcMotorEx.class, "shootL");
        gate = hardwareMap.get(Servo.class, "gate");
        rgbLight = hardwareMap.get(Servo.class, "rgbLight");
        hoodR = hardwareMap.get(Servo.class, "hoodR");
        hoodL = hardwareMap.get(Servo.class, "hoodL");

        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(19.8, 0, 0, 12.5960);
        shootR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        waitForStart(); //_______________________________________

        while (opModeIsActive()) { //_______________________________________
            if (gamepad2.right_bumper) {
                intakeStage1.setPower(1);
                intakeStage2.setPower(1);
            } else if (gamepad2.left_bumper) {
                intakeStage1.setPower(0);
                intakeStage2.setPower(0);
            }

            shootR.setVelocity(LimeLightConstants.Fly);
            shootL.setPower(shootR.getPower());

            hoodR.setPosition(LimeLightConstants.Hood);
            hoodL.setPosition(LimeLightConstants.Hood);

            //DRIVE
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            drive.drive(axial, lateral, yaw);
            //TELEMETRY DATA
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
