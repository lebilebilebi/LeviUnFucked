package org.firstinspires.ftc.teamcode.LebiPathing.TELEOPS;

import static org.firstinspires.ftc.teamcode.LebiPathing.robot.robot.OpModeType.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.LebiPathing.Subsystems.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.LebiPathing.robot.robot;
@TeleOp
public class teleOp extends CommandOpMode {
    private robot robot;
    private GamepadEx gp1;
    @Override
    public void initialize() {
        robot = new robot(telemetry, hardwareMap, TELEOP);
        gp1 = new GamepadEx(gamepad1);

        DriveCommand cmd = new DriveCommand(robot.drive);

        robot.schedule(
                new RunCommand(robot::read),
                new RunCommand(robot::loop),
                new RunCommand(robot::write)
        );
        robot.drive.setDefaultCommand(cmd.stickInputs(gp1));
    }
}
