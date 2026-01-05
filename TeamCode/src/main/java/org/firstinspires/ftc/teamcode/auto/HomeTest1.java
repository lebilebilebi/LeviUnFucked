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
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms;
import org.firstinspires.ftc.teamcode.Subsystems.InternalMechanisms.RoboStates;


@Autonomous(name = "Home Test 1", group = "Auto")
public class HomeTest1 extends OpMode {
    InternalMechanisms mechanisms;
    private DcMotorEx intake = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of robot
    private final Pose scorePose = new Pose(59.5, 84, Math.toRadians(180)); // Scoring pose
    private final Pose intakeStart1 = new Pose(20, 84); // Drive to first and intake first line (in this case, start intake when pathing to this pose)
    private final Pose intakeStart2 = new Pose(41, 60, Math.toRadians(180)); // Drive to second line
    private final Pose intake2 = new Pose(20, 60); // Intake along second line
    private final Pose intakeStart3 = new Pose(41, 36, Math.toRadians(180)); // Drive to third line
    private final Pose intake3 = new Pose(20, 36); // Intake along third line

    private PathChain scorePreload, driveAndIntake1, score1, drive2, intakePath2, score2, drive3, intakePath3, score3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        driveAndIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeStart1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeStart1.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart1, scorePose))
                .setLinearHeadingInterpolation(intakeStart1.getHeading(), scorePose.getHeading())
                .build();

        drive2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeStart2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intakeStart2.getHeading())
                .build();

        intakePath2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart2, intake2))
                .setLinearHeadingInterpolation(intakeStart2.getHeading(), intake2.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, scorePose))
                .setLinearHeadingInterpolation(intake2.getHeading(), scorePose.getHeading())
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
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(driveAndIntake1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(score1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(drive2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath2);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(score2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(drive3);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.INTAKE);
                    follower.followPath(intakePath3);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    mechanisms.setState(RoboStates.IDLE);
                    follower.followPath(score3);
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