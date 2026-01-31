package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;


@Autonomous(name = "DRIVE BLUE", group = "Auto")
public class DriveLine extends OpMode {
    InternalMechanisms mechanisms;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    double shootWaitTime = 5;
    private final Pose startPose = new Pose(60, 63.5, Math.toRadians(90)); // Start Pose of robot
    private final Pose scorePose = new Pose(60.22429906542056, 111.5, Math.toRadians(90)); // Ending pose    private final Pose endPose = new Pose(38.5, 13, Math.toRadians(90)); // Ending pose
    private final Pose endPose = new Pose(86.88785046728972, 111.83177570093459, Math.toRadians(-180)); // Scoring pose


    private PathChain driveToScore, driveToEnd;

    public void buildPaths() {
        driveToScore = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        driveToEnd = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mechanisms.setState(RoboStates.IDLE);
                follower.followPath(driveToScore);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SCORE);
                    if (pathTimer.getElapsedTimeSeconds() > 6) {
                        mechanisms.setState(RoboStates.IDLE);
                        follower.followPath(driveToEnd);
                        setPathState(-1);
                    }
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
        mechanisms.update();
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
        mechanisms = new InternalMechanisms(hardwareMap);
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