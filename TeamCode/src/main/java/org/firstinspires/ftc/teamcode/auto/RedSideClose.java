package org.firstinspires.ftc.teamcode.auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@Autonomous(name = "RED 12 BALL: CLOSE START", group = "Auto")
public class RedSideClose extends OpMode {
    InternalMechanisms mechanisms;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, scoreTimer; // Add scoreTimer
    private int pathState;
    private boolean scoringStarted = false;
    private static final double SCORE_DURATION = 1.2; // Scoring time constant
    double shootWaitTime = 1;
    private final Pose startPose = new Pose(25, 117, Math.toRadians(144)).mirror(); // Start Pose of robot
    private final Pose goalPose = new Pose(12, 137, Math.toRadians(140)).mirror(); // Pose of the goal
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(144)).mirror(); // Scoring pose
    private final Pose endPose = new Pose(20, 70, Math.toRadians(90)).mirror(); // Ending pose
    private final Pose turn = new Pose(58, 90, Math.toRadians(180)).mirror(); //Turn from preload shoot to face balls for first intake
    private final Pose intake1 = new Pose(29, 90, Math.toRadians(180)).mirror(); // Drive to first and intake first line (in this case, start intake when pathing to this pose)
    private final Pose intakeStart2 = new Pose(56, 66, Math.toRadians(180)).mirror(); // Drive to second line
    private final Pose intake2 = new Pose(20, 64, Math.toRadians(180)).mirror(); // Intake along second line
    private final Pose intakeStart3 = new Pose(56, 44, Math.toRadians(180)).mirror(); // Drive to third line
    private final Pose intake3 = new Pose(20, 44, Math.toRadians(180)).mirror(); // Intake along third line

    private PathChain scorePreload, driveToEnd, alignToIntake1, intakePath1, avoid, score1, drive2, intakePath2, score2, drive3, intakePath3, score3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        alignToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, turn))
                .setLinearHeadingInterpolation(scorePose.getHeading(), turn.getHeading())
                .build();

        intakePath1 = follower.pathBuilder()
                .addPath(new BezierLine(turn, intake1))
                .setLinearHeadingInterpolation(turn.getHeading(), intake1.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, scorePose))
                .setLinearHeadingInterpolation(intake1.getHeading(), scorePose.getHeading())
                .build();

        drive2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeStart2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeStart2.getHeading())
                .build();

        intakePath2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart2, intake2))
                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intake2.getHeading())
                .build();

        avoid = follower.pathBuilder()
                .addPath(new BezierLine(intake2, intakeStart2))
                .setLinearHeadingInterpolation(intake2.getHeading(), intakeStart2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart2, scorePose))
                .setLinearHeadingInterpolation(intakeStart2.getHeading(), scorePose.getHeading())
                .build();

        drive3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeStart3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeStart3.getHeading())
                .build();

        intakePath3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart3, intake3))
                .setLinearHeadingInterpolation(intakeStart3.getHeading(), intake3.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3, scorePose))
                .setLinearHeadingInterpolation(intake3.getHeading(), scorePose.getHeading())
                .build();

        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mechanisms.setState(RoboStates.IDLE);
                follower.followPath(scorePreload, true); // holdEnd = true
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (!scoringStarted) {
                        mechanisms.setState(RoboStates.AUTO_SCORE);
                        scoreTimer.resetTimer(); // Reset scoring timer when scoring starts
                        scoringStarted = true;
                    }
                    if (scoreTimer.getElapsedTimeSeconds() > SCORE_DURATION) {
                        mechanisms.setState(RoboStates.IDLE);
                        follower.followPath(alignToIntake1);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath1, 0.5, false);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(score1, true); // holdEnd = true
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (!scoringStarted) {
                        mechanisms.setState(RoboStates.AUTO_SCORE);
                        scoreTimer.resetTimer();
                        scoringStarted = true;
                    }
                    if (scoreTimer.getElapsedTimeSeconds() > SCORE_DURATION) {
                        mechanisms.setState(RoboStates.IDLE);
                        follower.followPath(drive2);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath2, 0.5, false);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(avoid);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(score2, true); // holdEnd = true
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    if (!scoringStarted) {
                        mechanisms.setState(RoboStates.AUTO_SCORE);
                        scoreTimer.resetTimer();
                        scoringStarted = true;
                    }
                    if (scoreTimer.getElapsedTimeSeconds() > SCORE_DURATION) {
                        mechanisms.setState(RoboStates.IDLE);
                        follower.followPath(drive3);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath3, 0.5, false);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(score3, true); // holdEnd = true
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (!scoringStarted) {
                        mechanisms.setState(RoboStates.AUTO_SCORE);
                        scoreTimer.resetTimer();
                        scoringStarted = true;
                    }
                    if (scoreTimer.getElapsedTimeSeconds() > SCORE_DURATION) {
                        mechanisms.setState(RoboStates.FULL_IDLE);
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
        scoringStarted = false; // Reset the flag when changing states
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        mechanisms.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("scoringStarted", scoringStarted);
        telemetry.addData("flywheel velocity", mechanisms.getVelocity());
        telemetry.addData("gateTimer", mechanisms.gateTimer.seconds());
        telemetry.addData("scoreTimer", scoreTimer.getElapsedTimeSeconds()); // Add this
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
        scoreTimer = new Timer(); // Initialize scoreTimer
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

