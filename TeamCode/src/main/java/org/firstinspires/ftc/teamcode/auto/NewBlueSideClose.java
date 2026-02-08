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
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;


@Autonomous(name = "NEW BLUE 12 BALL: CLOSE START", group = "NEW Auto")
public class NewBlueSideClose extends OpMode {
    // Subsystems
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private Timer pathTimer, opmodeTimer, scoreTimer;
    private int pathState;
    private boolean scoringStarted = false;
    private boolean feedPending = false;

    private static final double SCORE_DURATION = 1.2;
    double shootWaitTime = 1;

    private final Pose startPose = new Pose(21, 122, Math.toRadians(144));
    private final Pose scorePose = new Pose(59.5, 84, Math.toRadians(220));
    private final Pose spike1Intake = new Pose(16.5, 84, Math.toRadians(180));
    private final Pose spike2Intake = new Pose(19, 60);
    private final Pose spike2ControlPoint = new Pose(49.3924914675768, 59.22184300341295);
    private final Pose spike3Intake = new Pose(16, 35.5);
    private final Pose spike3ControlPoint = new Pose(52.96261682242991, 35.644859813084125);
    private final Pose gateControlPoint = new Pose(31.628504672897204, 58.224299065420574);
    private final Pose gateOpen = new Pose(27, 59);
    private final Pose gateIntake = new Pose(9, 55.5, Math.toRadians(95));
    private final Pose endPose = new Pose(20, 70, Math.toRadians(90));
    private PathChain scorePreload, spike1IntakePath, spike1ScorePath, spike2IntakePath, spike2ScorePath, gateIntakePath, gateScorePath, spike3IntakePath, spike3ScorePath, driveToEnd;

    // Helper methods to set combined states
    private void setIdle() {
        shooter.setState(Shooter.ShooterStates.IDLE);
        intake.setState(Intake.IntakeStates.IDLE);
    }

    private void setIntaking() {
        shooter.setState(Shooter.ShooterStates.IDLE);
        intake.setState(Intake.IntakeStates.INTAKE);
    }

    private void setSpinUp() {
        shooter.setState(Shooter.ShooterStates.SPIN_UP);
        intake.setState(Intake.IntakeStates.IDLE);
    }

    private void setFiring() {
        shooter.setState(Shooter.ShooterStates.FIRING);   // opens gate immediately
        intake.setState(Intake.IntakeStates.IDLE);        // wait for gate to open
        feedPending = true;
        scoreTimer.resetTimer();                          // reuse for gate delay
    }

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.5, () -> setSpinUp())
                .addParametricCallback(1.0, () -> setFiring())
                .addParametricCallback(1.0, () -> pathTimer.resetTimer())
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        spike2IntakePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.4, () -> setIntaking())
                .addPath(new BezierCurve(scorePose, spike2ControlPoint, spike2Intake))
                .setTangentHeadingInterpolation()
                .build();

        spike2ScorePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.3, () -> setSpinUp())
                .addParametricCallback(1.0, () -> setFiring())
                .addParametricCallback(1.0, () -> pathTimer.resetTimer())
                .addPath(new BezierLine(spike2Intake, scorePose))
                .setLinearHeadingInterpolation(spike2Intake.getHeading(), scorePose.getHeading())
                .build();

        gateIntakePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.9, () -> setIntaking())
                .addPath(new BezierCurve(scorePose, gateControlPoint, gateOpen))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(gateOpen, gateIntake))
                .setLinearHeadingInterpolation(gateOpen.getHeading(), gateIntake.getHeading())
                .build();

        gateScorePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.3, () -> setSpinUp())
                .addParametricCallback(1.0, () -> setFiring())
                .addParametricCallback(1.0, () -> pathTimer.resetTimer())
                .addPath(new BezierLine(gateIntake, scorePose))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), scorePose.getHeading())
                .build();

        spike1IntakePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.2, () -> setIntaking())
                .addPath(new BezierLine(scorePose, spike1Intake))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spike1Intake.getHeading())
                .build();

        spike1ScorePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.3, () -> setSpinUp())
                .addParametricCallback(1.0, () -> setFiring())
                .addParametricCallback(1.0, () -> pathTimer.resetTimer())
                .addPath(new BezierLine(spike1Intake, scorePose))
                .setLinearHeadingInterpolation(spike1Intake.getHeading(), scorePose.getHeading())
                .build();

        spike3IntakePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.5, () -> setIntaking())
                .addPath(new BezierCurve(scorePose, spike3ControlPoint, spike3Intake))
                .setTangentHeadingInterpolation()
                .build();

        spike3ScorePath = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addParametricCallback(0.3, () -> setSpinUp())
                .addParametricCallback(1.0, () -> setFiring())
                .addParametricCallback(1.0, () -> pathTimer.resetTimer())
                .addPath(new BezierLine(spike3Intake, scorePose))
                .setLinearHeadingInterpolation(spike3Intake.getHeading(), scorePose.getHeading())
                .build();

        driveToEnd = follower.pathBuilder()
                .addParametricCallback(0.0, () -> setIdle())
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                        follower.followPath(spike2IntakePath, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(spike2ScorePath, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.2){
                        follower.followPath(gateIntakePath, true);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(gateScorePath, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                        follower.followPath(gateIntakePath, true);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(gateScorePath, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                        follower.followPath(spike1IntakePath, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(spike1ScorePath, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                        follower.followPath(spike3IntakePath, true);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(spike3ScorePath, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                        follower.followPath(driveToEnd, true);
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

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Update subsystems every loop
        shooter.update();
        intake.update();

        // Handle gate-open delay before feeding
        if (shooter.getState() == Shooter.ShooterStates.FIRING) {
            if (feedPending && shooter.isGateOpen() && scoreTimer.getElapsedTimeSeconds() >= 0.3) {
                intake.setState(Intake.IntakeStates.FEED_SHOOTER);
                feedPending = false;
            }
        } else {
            feedPending = false;
        }

        // Update path state machine
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Shooter Velocity", "%.1f / %.1f tps", shooter.getCurrentVelocity(), shooter.getTargetVelocity());
        telemetry.addData("Intake Current", "%.2f / %.2f A", intake.getStage1Current(), intake.getStage2Current());
        telemetry.addData("Score Timer", "%.2f s", scoreTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        scoreTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);

        // Initialize subsystems
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // Build paths (must be after subsystems are initialized for callbacks)
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Nothing needed here
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
}
