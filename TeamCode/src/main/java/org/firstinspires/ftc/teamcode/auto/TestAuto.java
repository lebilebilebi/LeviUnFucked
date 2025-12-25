package org.firstinspires.ftc.teamcode.auto;

import androidx.viewpager.widget.PagerTitleStrip;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;

@Autonomous
public class TestAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer, autoTimer;

    public enum PathState{
        DRIVE_1,
        SHOOT_1
    }

    PathState pathState;

    private final Pose startPose = new Pose(35.21495327102804,10.093457943925227, Math.toRadians(90));
    private final Pose shootPose = new Pose(35.21495327102804,35.66355140186916, Math.toRadians(90));

    private PathChain drive1;

    public void buildPaths(){
        drive1=follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_1:
                follower.followPath(drive1, true);
                setPathState(pathState.SHOOT_1);
                break;
            case SHOOT_1:
                if(!follower.isBusy()){
                    telemetry.addLine("Done path 1");
                }
            break;
            default:
                telemetry.addLine("No State");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_1;
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // INIT other init mechanism (flywheel etc)

        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void start(){
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
    }



}