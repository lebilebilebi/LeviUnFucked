package org.firstinspires.ftc.teamcode.LebiPathing.AUTOS;

import static org.firstinspires.ftc.teamcode.LebiPathing.robot.robot.OpModeType.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.LebiPathing.Subsystems.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.LebiPathing.robot.robot;
import org.firstinspires.ftc.teamcode.LebiPathing.util.Point;

@Autonomous
public class AutoTest extends CommandOpMode {

    /*1: Start by creating poses.
    * 2: I made it so you don't HAVE to pass in anything, with default values for Point(); being 0, 0, 0.
    * 3: Add in the X, Y, and Heading of the point.
    * 4: OPTIONAL: by using .setThresholds(), you can pass in the radius and heading thresholds into the points.
    * */
    private final Point startPose = new Point(0, 0, 0);
    private final Point scorePose = new Point(24, 0, 0);
    private final Point pickupPose = new Point(24, 24, 0, 0.5).setThresholds(
            1.5, 10);


    /*1: Init it all: create a "public void initialize()", this should hold recalibration,
    * setting a start pose, and the setting up the robot class (pass in AUTO not TELEOP)
    */
    @Override
    public void initialize() {
        robot robot = new robot(telemetry, hardwareMap, AUTO);
        robot.drive.setLocation(startPose);
        robot.drive.recalibrateIMU();

        /*1: set up the DriveCommand class, I like to name it squid be cause then it makes the sentence
        * "squid to point" which is what we use:)
        * 2: create the 3 Run commands with the Runnable being robot (which you made before), and the
        * requirements being the read loop and write that make everything work.
        * 3: You can either run either Sequential or parallel command groups. Inside each of these,
        * you can use the drive command we defined with toPoint(), and it expects a x, y, and heading
        * however because we already defined the poses before, just use that because it makes it easier
        * to read and reuse points like scorePose for example.
        * */

        DriveCommand squid = new DriveCommand(robot.drive);

        robot.schedule(
                new RunCommand(robot::read),
                new RunCommand(robot::loop),
                new RunCommand(robot::write),
                new SequentialCommandGroup(
                        squid.toPoint(scorePose),
                        new WaitCommand(100),
                        squid.toPoint(pickupPose))
        );
    }
}
/* but wait why no super.run()? Because its already baked into the framework in a sense, so calling
* it externally is pointless in this case
* In the greatest pathing ever (LebiPathing), only call super.run() if you override run().
 */