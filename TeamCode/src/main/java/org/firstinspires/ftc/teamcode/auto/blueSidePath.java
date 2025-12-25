package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class blueSidePath extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Timer for state machine delays
    private Timer pathTimer = new Timer();
    
    public static int pathEndTimeout = 500;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.860, 123.364, Math.toRadians(143)));

        paths = new Paths(follower);
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Timer", pathTimer.getElapsedTime());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start immediately
                follower.followPath(paths.Path1, true);
                setPathState(1);
                break;
            case 1:
                // Wait for Path 1 to finish AND for the timer to exceed the timeout
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > pathEndTimeout) {
                    setPathState(-1);
                }
                break;
            case -1:
                telemetry.addData("Status", "Finished");
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.860, 123.364), new Pose(59.888, 84.112)))
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();
            Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(59.888, 84.112), new Pose(21.533, 83.664))).setTangentHeadingInterpolation().build();
            Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.533, 83.664), new Pose(46.654, 97.121))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140)).build();
            Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(46.654, 97.121), new Pose(41.271, 59.888))).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180)).build();
            Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(41.271, 59.888), new Pose(21.308, 59.888))).setTangentHeadingInterpolation().build();
            Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.308, 59.888), new Pose(53.832, 89.495))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140)).build();
            Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(53.832, 89.495), new Pose(40.598, 35.439))).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180)).build();
            Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(40.598, 35.439), new Pose(21.533, 35.439))).setTangentHeadingInterpolation().build();
            Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(21.533, 35.439), new Pose(54.505, 89.047))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140)).build();
        }
    }
}