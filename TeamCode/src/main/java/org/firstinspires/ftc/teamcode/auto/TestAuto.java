package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class TestAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.186915887850464, 122.69158878504673, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.187, 122.692), new Pose(59.215, 83.888))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.215, 83.888), new Pose(15.925, 84.336))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.925, 84.336), new Pose(47.551, 96.449))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.551, 96.449), new Pose(41.944, 59.888))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.944, 59.888), new Pose(20.636, 59.888))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.636, 59.888), new Pose(47.551, 96.224))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.551, 96.224), new Pose(40.374, 35.888))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.374, 35.888), new Pose(21.533, 36.112))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.533, 36.112), new Pose(47.327, 96.224))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.327, 96.224), new Pose(31.402, 10.318))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(31.402, 10.318), new Pose(12.336, 10.318))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.336, 10.318), new Pose(47.551, 96.449))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    follower.followPath(paths.Path1);
                    return 1;
                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path2);
                        return 2;
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path3);
                        return 3;
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path4);
                        return 4;
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path5);
                        return 5;
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path6);
                        return 6;
                    }
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path7);
                        return 7;
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path8);
                        return 8;
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path9);
                        return 9;
                    }
                    break;
                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path10);
                        return 10;
                    }
                    break;
                case 10:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path11);
                        return 11;
                    }
                    break;
                case 11:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path12);
                        return 12;
                    }
                    break;
                case 12:
                    if (!follower.isBusy()) {
                        // Autonomous finished
                        return -1;
                    }
                    break;
            }
            return pathState; // If no transition, stay in current state
        }

    }
