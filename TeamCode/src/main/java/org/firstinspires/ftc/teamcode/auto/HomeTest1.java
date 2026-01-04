package org.firstinspires.ftc.teamcode.auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Home Test 1", group = "Auto")
public class HomeTest1 extends OpMode {
    private DcMotorEx intake = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose startInake1 = new Pose(35, 40, Math.toRadians(90)); // Start of intake Pose
    private final Pose intake1 = new Pose(35, 65, Math.toRadians(90)); // End of intake pose (Drive over balls to intake)
    private final Pose scorePose = new Pose(10, 60, Math.toRadians(-90)); // Scoring pose
    private Path scorePreload;
    private PathChain driveToBall, intakeBalls, scoreBalls;

    public void buildPaths() {
        driveToBall = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startInake1))
                .setLinearHeadingInterpolation(startPose.getHeading(), startInake1.getHeading())
                .build();

        intakeBalls = follower.pathBuilder()
                .addPath(new BezierLine(startInake1, intake1))
                .setLinearHeadingInterpolation(startInake1.getHeading(), intake1.getHeading())
                .build();

        scoreBalls = follower.pathBuilder()
                .addPath(new BezierLine(intake1, scorePose))
                .setLinearHeadingInterpolation(intake1.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(driveToBall);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(intakeBalls);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(scoreBalls, true);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "int"); //intake motor init statement
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}