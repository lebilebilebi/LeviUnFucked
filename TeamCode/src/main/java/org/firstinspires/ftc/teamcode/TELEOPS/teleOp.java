package org.firstinspires.ftc.teamcode.TELEOPS;

import static org.firstinspires.ftc.teamcode.robot.robot.OpModeType.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsystems.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.robot;
@TeleOp
public class teleOp extends CommandOpMode {
    private robot robot;
    private GamepadEx gp1;
    @Override
    public void initialize() {
        robot = new robot(telemetry, hardwareMap, TELEOP);
        gp1 = new GamepadEx(gamepad1);

        robot.schedule(
                new RunCommand(robot::read),
                new RunCommand(robot::loop),
                new RunCommand(robot::write)
        );
        robot.drive.setDefaultCommand(new DriveCommand.StickInputs(robot.drive, gp1));
    }
}
