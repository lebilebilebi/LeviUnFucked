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
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@Autonomous(name = "NEW BLUE 12 BALL: CLOSE START", group = "NEW Auto")
public class NewBlueSideClose extends OpMode {
    InternalMechanisms mechanisms;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, scoreTimer; // Add scoreTimer
    private int pathState;
    private boolean scoringStarted = false;
    private static final double SCORE_DURATION = 1.2; // Scoring time constant
    double shootWaitTime = 1;
    private final Pose startPose = new Pose(21, 122, Math.toRadians(144)); // Start Pose of robot
    private final Pose scorePose = new Pose(59.5, 84, Math.toRadians(220)); // Scoring pose
    private final Pose spike1Intake = new Pose(16.5, 84, Math.toRadians(180)); // Drive to first and intake first line (in this case, start intake when pathing to this pose)
    private final Pose spike2Intake = new Pose(19, 60); // Drive to first and intake first line (in this case, start intake when pathing to this pose)
    private final Pose spike2ControlPoint = new Pose(49.3924914675768, 59.22184300341295);
    private final Pose spike3Intake = new Pose(16, 35.5); // Drive to first and intake first line (in this case, start intake when pathing to this pose)
    private final Pose spike3ControlPoint = new Pose(52.96261682242991, 35.644859813084125);
    private final Pose gateControlPoint = new Pose(31.628504672897204, 58.224299065420574);
    private final Pose gateOpen = new Pose(27, 59); // Intake along second line
    private final Pose gateIntake = new Pose(9, 55.5, Math.toRadians(95)); // Intake along second line
    private final Pose endPose = new Pose(20, 70, Math.toRadians(90)); // Ending pose
    private PathChain scorePreload, spike1IntakeAndScore, spike2IntakeAndScore, gateIntakeAndScore, spike3IntakeAndScore, driveToEnd;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        spike2IntakeAndScore = follower.pathBuilder()
                //.addParametricCallback(0.0, ()->{mechanisms.setState(RoboStates.INTAKE);})
                .addPath(new BezierCurve(scorePose, spike2ControlPoint, spike2Intake))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(spike2Intake, scorePose))
                .setLinearHeadingInterpolation(spike2Intake.getHeading(), scorePose.getHeading())
                .build();

        gateIntakeAndScore = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, gateControlPoint, gateOpen))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(gateOpen, gateIntake))
                .setLinearHeadingInterpolation(gateOpen.getHeading(), gateIntake.getHeading())
                .addPath(new BezierLine(gateIntake, scorePose))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), scorePose.getHeading())
                .build();

        spike1IntakeAndScore = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, spike1Intake))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spike1Intake.getHeading())
                .addPath(new BezierLine(spike1Intake, scorePose))
                .setLinearHeadingInterpolation(spike1Intake.getHeading(), scorePose.getHeading())
                .build();

        spike3IntakeAndScore = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, spike3ControlPoint, spike3Intake))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(spike3Intake, scorePose))
                .setLinearHeadingInterpolation(spike3Intake.getHeading(), scorePose.getHeading())
                .build();

        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true); // holdEnd = true
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(spike2IntakeAndScore, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntakeAndScore, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntakeAndScore, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(spike1IntakeAndScore, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(spike3IntakeAndScore, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(driveToEnd, true);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        scoringStarted = false;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        mechanisms.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("scoringStarted", scoringStarted);
        telemetry.addData("gateTimer", mechanisms.gateTimer.seconds());
        telemetry.addData("scoreTimer", scoreTimer.getElapsedTimeSeconds());
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
        scoreTimer = new Timer();
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
