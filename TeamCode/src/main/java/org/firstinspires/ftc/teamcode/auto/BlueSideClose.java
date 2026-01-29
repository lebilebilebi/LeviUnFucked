package org.firstinspires.ftc.teamcode.auto;
import com.pedropathing.follower.Follower;
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


@Autonomous(name = "BLUE 12 BALL: CLOSE START", group = "Auto")
public class BlueSideClose extends OpMode {
    InternalMechanisms mechanisms;
    private DcMotorEx intake = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    double shootWaitTime = 5;
    private final Pose startPose = new Pose(21.45794392523365, 122.61682242990655, Math.toRadians(140)); // Start Pose of robot
    private final Pose goalPose = new Pose(12, 137, Math.toRadians(140)); // Pose of the goal
    private final Pose scorePose = new Pose(55, 88, Math.toRadians(144)); // Scoring pose
    private final Pose endPose = new Pose(13, 70, Math.toRadians(90)); // Ending pose
    private final Pose turnTo180 = new Pose(55, 84, Math.toRadians(180)); //Turn from preload shoot to face balls for first intake
    private final Pose intakeStart1 = new Pose(9, 84, Math.toRadians(180)); // Drive to first and intake first line (in this case, start intake when pathing to this pose)
    private final Pose intakeStart2 = new Pose(41, 58, Math.toRadians(180)); // Drive to second line
    private final Pose intake2 = new Pose(8, 58, Math.toRadians(180)); // Intake along second line
    private final Pose intakeStart3 = new Pose(41, 36, Math.toRadians(180)); // Drive to third line
    private final Pose intake3 = new Pose(8, 36, Math.toRadians(180)); // Intake along third line

    private PathChain scorePreload, driveToEnd, turn, driveAndIntake1, score1, drive2, intakePath2, score2, drive3, intakePath3, score3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        turn = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(scorePose, turnTo180))
                .setLinearHeadingInterpolation(scorePose.getHeading(), turnTo180.getHeading())
                .build();

        driveAndIntake1 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(turnTo180, intakeStart1))
                .setLinearHeadingInterpolation(turnTo180.getHeading(), intakeStart1.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(intakeStart1, scorePose))
                .setLinearHeadingInterpolation(intakeStart1.getHeading(), scorePose.getHeading())
                .build();

        drive2 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(scorePose, intakeStart2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeStart2.getHeading())
                .build();

        intakePath2 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(intakeStart2, intake2))
                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intake2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(intake2, scorePose))
                .setLinearHeadingInterpolation(intake2.getHeading(), scorePose.getHeading())
                .build();

        drive3 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(scorePose, intakeStart3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeStart3.getHeading())
                .build();

        intakePath3 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(intakeStart3, intake3))
                .setLinearHeadingInterpolation(intakeStart3.getHeading(), intake3.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(intake3, scorePose))
                .setLinearHeadingInterpolation(intake3.getHeading(), scorePose.getHeading())
                .build();

        driveToEnd = follower.pathBuilder()
                .addParametricCallback(1.0,()->{pathTimer.resetTimer();})
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mechanisms.setState(RoboStates.AUTO_SPINUP);
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SCORE);
                    if (pathTimer.getElapsedTimeSeconds() > shootWaitTime) {
                        mechanisms.setState(RoboStates.IDLE);
                        follower.followPath(turn);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(driveAndIntake1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SPINUP);
                    follower.followPath(score1);
                    setPathState(4);
                    }
                break;

            case 4:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SCORE);
                    if (pathTimer.getElapsedTimeSeconds() > shootWaitTime) {
                        follower.followPath(drive2);
                        setPathState(5);
                        mechanisms.setState(RoboStates.IDLE);
                        break;
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SPINUP);
                    follower.followPath(score2);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SCORE);
                    if (pathTimer.getElapsedTimeSeconds() > shootWaitTime) {
                        follower.followPath(drive3);
                        setPathState(8);
                        mechanisms.setState(RoboStates.IDLE);
                        break;
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath3);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SPINUP);
                    follower.followPath(score3);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    mechanisms.setState(RoboStates.AUTO_SCORE);
                    if (pathTimer.getElapsedTimeSeconds() > shootWaitTime) {
                        follower.followPath(driveToEnd);
                        setPathState(-1);
                        mechanisms.setState(RoboStates.IDLE);
                        break;
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

        // Update distance to goal based on current robot position
        mechanisms.updateGoalDist(follower.getPose().getX(), follower.getPose().getY());

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("distance to goal", mechanisms.getGoalDist());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        mechanisms = new InternalMechanisms(hardwareMap);
        mechanisms.setGoalPose(goalPose.getX(), goalPose.getY());
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