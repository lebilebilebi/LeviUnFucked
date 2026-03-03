package org.firstinspires.ftc.teamcode.AUTOS;

import static org.firstinspires.ftc.teamcode.robot.robot.OpModeType.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.robot.robot;
import org.firstinspires.ftc.teamcode.util.Point;

@Autonomous
public class AutoTest extends CommandOpMode {
    @Override
    public void initialize() {
        robot robot = new robot(telemetry, hardwareMap, AUTO);
        robot.drive.setLocation(new Point());
        robot.drive.recalibrateIMU();
        robot.schedule(
                new RunCommand(robot::read),
                new RunCommand(robot::loop),
                new RunCommand(robot::write),
                new RunCommand(robot.drive::driveToTarget),
                new SequentialCommandGroup(
                        new DriveCommand.DriveToPoint(robot.drive,
                                new Point(24, 0, 0)),
                        new DriveCommand.DriveToPoint(robot.drive,
                                new Point(24, 24, 0)))
        );
    }
}
