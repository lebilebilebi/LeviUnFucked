package org.firstinspires.ftc.teamcode.LebiPathing.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.LebiPathing.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.LebiPathing.util.Point;

@TeleOp
@Config
public class HeadingTuner extends CommandOpMode {
    private Drive drive;
    public static double sQH;
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); {
        }
        this.drive = new Drive(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftRear"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "rightRear"),
                hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint"),
                telemetry,
                null
        );

        CommandScheduler.getInstance().schedule(
                new RunCommand(drive::read),
                new RunCommand(drive::loop),
                new RunCommand(drive::write)
        );

        drive.setLocation(new Point());
        drive.recalibrateIMU();
    }

    @Override
    public void run() {
        drive.stickInput(0, 0,0);
        if (gamepad1.a){
            drive.setTargetPoint(new Point());
            drive.driveToTarget();
        } else if (gamepad1.b) {
            drive.setTargetPoint(new Point(0, 0, 180));
            drive.driveToTarget();
        }
        drive.tuneHeading(sQH);
        telemetry.update();
        super.run();
    }
}